/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <bgp_calib/calib_tool.h>
#include <bgp_calib/utils.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/AntiFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <Eigen/Eigenvalues>
#include <boost/filesystem.hpp>
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>

using std::cout;
using std::endl;

namespace bgp_calib {
  typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> ProjectionFactor;

  static std::vector<gtsam::Point3> wps;  // for debugging
  static std::vector<std::pair<double, double>> ips; // for debugging

  static int to_point_id(int tagid, int corner) {
    return (tagid * 4 + corner);
  }
  static gtsam::Symbol getCamToWorldSym(int camid, int frame) {
    return (gtsam::Symbol('c' + camid, frame));
  }
  static gtsam::Symbol getWSym(int tagid, int corner) {
    return (gtsam::Symbol('w', 4 * tagid + corner));
  }
  static gtsam::Symbol getOSym(int tagid) {
    return (gtsam::Symbol('o', tagid));
  }

  static void getTagAndCornerFromIndex(int index, int *tagid, int *corner) {
    *corner = index % 4;
    *tagid = index / 4;
  }

  static std::ofstream open_file(const std::string &filename) {
    boost::filesystem::path fname(filename);
    boost::filesystem::create_directories(fname.parent_path());
    return (std::ofstream(filename));
  }

  static void print_pose(std::ostream &of, const gtsam::Pose3 &p) {
    gtsam::Matrix4 M = p.matrix();
    of << "    [" << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << ", " << M(0,3) <<","<< endl;
    of << "     " << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << ", " << M(1,3) <<","<< endl;
    of << "     " << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << ", " << M(2,3) <<","<< endl;
    of << "     " << M(3,0) << ", " << M(3,1) << ", " << M(3,2) << ", " << M(3,3) <<"]"<< endl;
  }
  
  static void print_pose_yaml(std::ostream &of, const gtsam::Pose3 &p) {
    gtsam::Matrix4 M = p.matrix();
    of << std::setprecision(8) << std::fixed;
    of << "  - [" << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << ", " << M(0,3) <<"]"<< endl;
    of << "  - [" << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << ", " << M(1,3) <<"]"<< endl;
    of << "  - [" << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << ", " << M(2,3) <<"]"<< endl;
    of << "  - [" << M(3,0) << ", " << M(3,1) << ", " << M(3,2) << ", " << M(3,3) <<"]"<< endl;
  }

  static std::string to_string(std::string name, const gtsam::Pose3 &p) {
    std::stringstream ss;
    ss << name << " = [..." <<  endl;
    const auto H = p.matrix();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        ss << ", " << H(i,j);
      }
      if (i < 3) {
        ss << "; ..." << endl;
      } else {
        ss << "];" << endl;
      }
    }
    return (ss.str());
  }
  
  static boost::shared_ptr<gtsam::Cal3DS2> makeCameraModel(CamPtr c) {
    const Eigen::Matrix<double, 3, 3> &K = c->K();
    const Eigen::Matrix<double, 1, 5> &D = c->D();
    //Cal3DS2 (fx, fy, skew, u0, v0, k1, k2, p1, p2);
    //D(from ROS) has k1 k2 p1 p2 k3  (p1/p2 are tangential distortion)
    boost::shared_ptr<gtsam::Cal3DS2>
      model(new gtsam::Cal3DS2(K(0,0),K(1,1),0, K(0,2),K(1,2),
                               D(0,0),D(0,1),D(0,2),D(0,3)));
    return (model);
  }

//#define DEBUG_PRINT

  static gtsam::Pose3 guessPose(CamPtr c, const Tag &tag,
                                const std::vector<gtsam::Point2> ip) {
    std::vector<gtsam::Point3> wp;  // image points
    for (int i = 0; i < 4; i++) {   // loop over corners
      wp.push_back(tag.getObjectCorner(i));
#ifdef DEBUG_PRINT
      cout << ip[i].x() << " " << ip[i].y() << " "
           << wp.back().x() << " " << wp.back().y() << " " << wp.back().z() << endl;
#endif
      gtsam::Point3 wp0(wp.back().x(), wp.back().y(), 0.0);
      gtsam::Point3 wpt = tag.pose.transform_from(wp0);

      wps.push_back(wpt);
      ips.push_back(std::pair<double,double>(ip[i].x(), ip[i].y()));
    }
#ifdef DEBUG_PRINT
    cout << "K: " << c->K() << endl;
#endif    

    gtsam::Pose3 guess = utils::get_init_pose(wp, ip, c->K());
    return (guess);
  }
  
  static double tryOptimization(gtsam::Values *result,
                                const gtsam::NonlinearFactorGraph &graph,
                                const gtsam::Values &values,
                                const std::string &verbosity, int maxIter) {
      gtsam::LevenbergMarquardtParams lmp;
      lmp.setVerbosity(verbosity);
      lmp.setMaxIterations(maxIter);
      lmp.setAbsoluteErrorTol(1e-7);
      lmp.setRelativeErrorTol(0);
      gtsam::LevenbergMarquardtOptimizer lmo(graph, values, lmp);
      *result = lmo.optimize();
      double err = lmo.error();
      return (err);
  }

  static void printCameraPoses(const gtsam::Values &values) {
    gtsam::Values::const_iterator it;
    for (it = values.begin(); it != values.end(); ++it) {
      gtsam::Symbol sym(it->key);
      if (sym.chr() == 'c') {
        const gtsam::Pose3 &pose = values.at<gtsam::Pose3>(it->key);
        cout << "camera world pose: " << pose << endl;
      }
    }
  }

  void CalibTool::clear() {
    obsTags_.clear();
    values_.clear();
    graph_.erase(graph_.begin(), graph_.end());
    for (CamPtr &cam: cam_) {
      cam->clear();
    }

  }
  
  gtsam::Values CalibTool::optimize() {
#ifdef DEBUG_PRINT_OPT
    cout << "---------------- graph: ------------------- " << endl;
    graph_.print();
    cout << "---------------- values: ------------------- " << endl;
    values_.print();
    cout << "---------- output from optimizer ---------" << endl;
#endif
    double err = tryOptimization(&optimizedValues_, graph_, values_,
                                 "TERMINATION", /*num iter*/ 100);
    
    if (err > maxError_) {
      ROS_WARN("ERROR: camera calibration not converged!");
    }
#ifdef DEBUG_PRINT_OPT
    cout << "------------------------------------------" << endl;
    optimizedValues_.print();
    cout << "-------------- SUCCESS -----------------" << endl;
    printCameraPoses(result);
#endif    
    return (optimizedValues_);
  }



  gtsam::Symbol CalibTool::getObjectCoordSym(const Tag &tag, int corner) {
    gtsam::Symbol sym('x', tag.sizenumber * 4 + corner);
    if (values_.find(sym) == values_.end()) {
    	IsotropicNoisePtr objNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
      const gtsam::Point3 X = tag.getObjectCorner(corner);
      values_.insert(sym, X);
      graph_.push_back(gtsam::PriorFactor<gtsam::Point3>(sym, X, objNoise));
    }
    return (sym);
  }

  int CalibTool::tagIdToTagNumber(int tagid) const {
    std::map<int, Tag>::const_iterator it = tags_.find(tagid);
    if (it == tags_.end()) {
      ROS_ERROR("bad tag id: %d", tagid);
    }
    return (std::distance(tags_.begin(), it));
  }

  
  gtsam::Symbol CalibTool::getObjectToCamSym(int camid, int frame, int tagid) {
    int tagNum = tagIdToTagNumber(tagid);
    return (gtsam::Symbol('o' + camid,
                          frame * tags_.size() + tagNum));
  }

  void CalibTool::insertTagIfNew(int frame_num, int camid,
                                 const apriltag_msgs::Apriltag &otag) {
    if (obsTags_.count(frame_num) != 0
        && obsTags_[frame_num].count(camid) != 0
        && obsTags_[frame_num][camid].count(otag.id) != 0) {
      return;
    }
    // insert as observed tag!
    obsTags_[frame_num][camid][otag.id] = otag;
    // find tag
    const Tag &tag = tags_[otag.id];
    // add wTo transform and prior factor for it
    gtsam::Symbol osym(getOSym(tag.id));   // wTo
    if (!values_.exists(osym)) {
      graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(osym, tag.pose, tag.noise));
      values_.insert(osym, tag.pose);
      auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4); // error in m!
      // now add world coordinate points "w" to graph and values
      for (int i = 0; i < 4; i++) {   // loop over corners
        // this will also insert the object coordinates X if needed!
        gtsam::Symbol xsym = getObjectCoordSym(tag, i);
        // add ref frame factor and corresponding values
        gtsam::Symbol wsym = getWSym(tag.id, i);
        graph_.push_back(gtsam::ReferenceFrameFactor<gtsam::Point3,
                         gtsam::Pose3>(xsym, osym, wsym, noise));
        gtsam::Point3 oX = values_.at<gtsam::Point3>(xsym);
        values_.insert(wsym, tag.pose.transform_from(oX));
      }
    }
  }

  bool CalibTool::allCamerasHaveFrames() const {
    bool haveAllFrames(true);
    for (const CamPtr &cam: cam_) {
      if (!cam->gotFrames()) {
        haveAllFrames = false;
        break;
      }
    }
    return (haveAllFrames);
  }

  int CalibTool::frameObserved(const apriltag_msgs::ApriltagArrayStamped::ConstPtr &msg,
                               int camid) {
    const ros::Time &t = msg->header.stamp;
    if (camid < 0 || camid > cam_.size()) {
      ROS_ERROR("invalid cam id %d, total cams: %d", camid, (int)cam_.size());
      return (0);
    }
    CamPtr &c = cam_[camid];
    int frame_num = c->getFrameNum(t);
    
    int numUsed(0);
    for (const auto &tag: msg->apriltags) {
      if (tagObserved(frame_num, camid, tag)) {
        numUsed++;
      }
    }
    if (numUsed > 0) {
      if (!makeCameraPositionGuess(frame_num, camid)) {
        ROS_ERROR("cannot find camera position guess for frame %d cam %s", frame_num,
                  c->getName().c_str());
        return (0);
      }
    }
    ROS_INFO_STREAM("cam " << c->getName() << " processed " << numUsed << " of " <<
                    msg->apriltags.size() << " tags!");
    return (numUsed);
  }

  bool CalibTool::makeCameraPositionGuess(int frame_num, int camid) {
    gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
    CamPtr &c = cam_[camid];
    
    if (!values_.exists(csym)) {
      gtsam::Pose3 wTc = guessPoseFromHomographies(frame_num, camid);
      //cout << to_string("wTc", wTc);
      values_.insert(csym, wTc);
    }
    
    double  initSearchSize(0.5);
    RandEng	 randomEngine;
    RandDist distTrans(0, initSearchSize); // mu, sigma for trans
    RandDist distRot(0, M_PI);		         // mu, sigma for rotations
    RandGen	 rgt(randomEngine, distTrans);	 // random translation generator
    RandGen  rgr(randomEngine, distRot);	 // random angle generator

    const int numTries(100);
    const int maxIter(100);

    gtsam::Values bestValues;
    double err = tryOptimization(&bestValues, graph_, values_, "SILENT", maxIter);
    ROS_INFO_STREAM("cam " << c->getName() << " optimizer error on first guess try: " << err);
    if (err > maxError_) {
      ROS_WARN("initial guess for cam %s position had error of %f > %f,"
               " resorting to random guessing!", c->getName().c_str(),
               err, maxError_);
    }
    double minError(1e50);
    for (int i = 0; i < numTries && err > maxError_; i++) {
      // cannot get it done with reasonable starting guess,
      // resort to random guessing
      const gtsam::Pose3 rp = makeRandomPose(&rgr, &rgt);
      //cout << "using random pose: " << rp << endl;
      updateStartingPose(frame_num, camid, rp);
      gtsam::Values result;
      err = tryOptimization(&result, graph_, values_,
                            "SILENT", maxIter);
      if (err < minError) {
        bestValues = result;
        minError = err;
      }
    }
    if (err > maxError_) {
      ROS_WARN_STREAM("init cam position failed, best error was: " << minError <<
                       " > max error of "  << maxError_);
      ROS_WARN("consider bumping max_error!");
      values_ = bestValues; // use the best values we found
      return (false);
    }

    return (true);
  }

  static gtsam::Pose3 averagePose(const std::vector<gtsam::Pose3> &poses) {
    // This is a really dirty hack to average over rotations and positions.
    // Any outliers, who knows what happens. A better way would
    // be to compute a homography from all points that lie in a plane.
    if (poses.empty()) {
      return (gtsam::Pose3());
    }
    gtsam::Point3 sumTrans;
    Eigen::Matrix<double, 4, 4> qq = Eigen::Matrix<double, 4, 4>::Zero();

    for (const auto &p: poses) {
      sumTrans = sumTrans + p.translation();
      gtsam::Quaternion q = p.rotation().toQuaternion();
      Eigen::Matrix<double, 4, 1> qv{q.w(), q.x(), q.y(), q.z()};
      qq = qq + (qv * qv.transpose());
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 4, 4>> es(qq);
    Eigen::Matrix<double, 4, 4> evec = es.eigenvectors();
    int col = 3;
    gtsam::Quaternion qavg(evec(0,col), evec(1,col), evec(2,col), evec(3,col));
    gtsam::Pose3 avgPose(gtsam::Rot3(qavg), sumTrans / (double)poses.size());
    return (avgPose);
  }

  gtsam::Pose3 CalibTool::guessPoseFromHomographies(int frame_num, int camid) {
    CamPtr &c = cam_[camid];
    gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
    std::vector<gtsam::Pose3> poses;
    std::vector<gtsam::Point2> tagImagePoints;
    for (gtsam::NonlinearFactorGraph::const_iterator f = graph_.begin();
         f != graph_.end(); ++f) {
      gtsam::NonlinearFactor const &fac = *(*f);
      // iterate through all keys that are referred to
      for (gtsam::Factor::const_iterator it = fac.begin(); it != fac.end(); ++it) {
        gtsam::Symbol sym(*it);
        if (csym == sym) {
          auto it2 = it; it2++;
          gtsam::Symbol wsym(*it2);
          if (wsym.chr() == 'w') {
            int tagid(0), corner(0);
            getTagAndCornerFromIndex(wsym.index(), &tagid, &corner);
            const ProjectionFactor &pf = dynamic_cast<const ProjectionFactor &>(fac);
            tagImagePoints.push_back(pf.measured());
            if (tagImagePoints.size() == 4) {
              const Tag &tag = tags_[tagid];
              //
              // add guess for wTc = wTo * oTc
              //
              gtsam::Pose3 cTo = guessPose(c, tag, tagImagePoints);
              gtsam::Pose3 wTc = tag.pose.compose(cTo.inverse());
              poses.push_back(wTc);
              tagImagePoints.clear();
            }
          }
        }
      }
    }
    // now average over poses.
    gtsam::Pose3 avg = averagePose(poses);
    return (avg);
	}
  
  bool CalibTool::tagObserved(int frame_num,
                              int camid, const apriltag_msgs::Apriltag &obstag) {
    CamPtr &c = cam_[camid];
    if (tags_.count(obstag.id) == 0 || !c->tagAllowed(obstag.id)) {
      return (false);
    }
    //cout << "using tag: " << obstag.id << endl;

    const Tag &tag = tags_[obstag.id];

    insertTagIfNew(frame_num, camid, obstag);

    //
    // add projection factor
    //
    boost::shared_ptr<gtsam::Cal3DS2> camModel = makeCameraModel(c);
    IsotropicNoisePtr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 2.0);
    gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
    
    for (int i = 0; i < 4; i++) {   // loop over corners
      gtsam::Symbol wsym = getWSym(tag.id, i);
      gtsam::Point2 uv(obstag.corners[i].x, obstag.corners[i].y);
      graph_.push_back(ProjectionFactor(uv, pixelNoise, csym, wsym, camModel));
    }


//#define CLEAR_IT
#ifdef CLEAR_IT
    values_.clear();
    graph_.erase(graph_.begin(), graph_.end());
#endif
    return (true);
  }

  
  gtsam::Pose3 CalibTool::makeRandomPose(RandGen *rgr, RandGen *rgt) {
    gtsam::Point3 t((*rgt)(), (*rgt)(), (*rgt)());
    gtsam::Point3 om = gtsam::Point3((*rgr)(), (*rgr)(), (*rgr)());
    return (gtsam::Pose3(gtsam::Rot3::rodriguez(om.x(),om.y(),om.z()), gtsam::Point3(t)));
  }

  void CalibTool::updateStartingPose(int frame_num, int camid, const gtsam::Pose3 &wTc) {
    gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
    gtsam::Values::const_iterator it;
    const gtsam::Values &values = values_;
    for (it = values.begin(); it != values.end(); ++it) {
      gtsam::Symbol sym(it->key);
      if (sym == csym) {
        values_.update(sym, wTc);
      }
    }
  }

  void CalibTool::printResults() const {
    ROS_INFO_STREAM("======================= result ================");
    for (auto const &framekv: obsTags_) {
      int frame_num = framekv.first;
      ROS_INFO_STREAM(" --- frame: " << frame_num);
      for (auto const &camkv: framekv.second) {
        int camid = camkv.first;
        ROS_INFO_STREAM(" camera: " << cam_[camid]->getName());
        gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
        gtsam::Pose3 wTc = optimizedValues_.at<gtsam::Pose3>(csym);
        ROS_INFO_STREAM(to_string("wTc" + std::to_string(camid), wTc));
      }
    }
  }

  void CalibTool::writeCalibrationFile(const std::string &filename) const {
    std::ofstream of = open_file(filename);
    ROS_INFO_STREAM("writing extrinsic calibration to file " << filename);
    for (int camid = 0; camid < cam_.size(); camid++) {
      const auto &cam = *cam_[camid];
      int frame_num = 0;
      gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
      if (optimizedValues_.exists(csym)) {
        gtsam::Pose3 wTc = optimizedValues_.at<gtsam::Pose3>(csym);
        of << cam.getName() << ":" << std::endl;
        of << "  T_cam_imu:" << std::endl;
        print_pose_yaml(of, wTc.inverse());
        cam.print_intrinsics(of);
        of << "  rostopic: " << cam.getName() << std::endl;
      }
    }
  }

  void CalibTool::writeCameraPoses(const std::string &filename) const {
    ROS_INFO_STREAM("writing camera poses to file " << filename);
    std::ofstream of = open_file(filename);
    for (int camid = 0; camid < cam_.size(); camid++) {
      const auto &cam = *cam_[camid];
      int frame_num = 0;
      gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
      if (optimizedValues_.exists(csym)) {
        gtsam::Pose3 wTc = optimizedValues_.at<gtsam::Pose3>(csym);
        const gtsam::Point3 rvec = gtsam::Rot3::Logmap(wTc.rotation());
        gtsam::Point3 t = wTc.translation();
        of << camid << " " << rvec.x() << " " << rvec.y() << " " << rvec.z()
           << " " << t.x() << " " << t.y() << " " << t.z() << endl;
      }
    }
  }
  void CalibTool::writeTagPoses(const std::string &filename) const {
    ROS_INFO_STREAM("writing tag poses to file " << filename);
    std::ofstream of = open_file(filename);
    gtsam::Marginals marginals(graph_, optimizedValues_);
    gtsam::Values::const_iterator it;
    const gtsam::Values &values = optimizedValues_;
    for (it = values.begin(); it != values.end(); ++it) {
      gtsam::Symbol sym(it->key);
      if (sym.chr() == 'o') {
        int tagid = sym.index();
        gtsam::Matrix cov(6,6);
        cov = marginals.marginalCovariance(sym);

        const gtsam::Pose3 &p = values.at<gtsam::Pose3>(it->key);
        const gtsam::Point3 t = p.translation();
        const gtsam::Point3 r = gtsam::Rot3::Logmap(p.rotation());
        std::map<int, Tag>::const_iterator ti = tags_.find(tagid);
        if (ti != tags_.end()) {
          of << tagid << " " << ti->second.size << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << r.x() << " " << r.y() << " " << r.z() << " "
             << cov(3,3) << " " << cov(4,4) << " " << cov(5,5) << " "
             << cov(0,0) << " " << cov(1,1) << " " << cov(2,2) << endl;
        }
      }
    }
  }

  void CalibTool::writeReprojectionData(const std::string &filename) const {
    ROS_INFO_STREAM("writing reprojection data to file " << filename);
    std::ofstream of = open_file(filename);
    for (auto const &framekv: obsTags_) { // loop over all frames
      int frame_num = framekv.first;
      for (auto const &camkv: framekv.second) { // loop over all cameras
        int camid = camkv.first;
        const CamPtr &c = cam_[camid];

        boost::shared_ptr<gtsam::Cal3DS2> camModel = makeCameraModel(c);

        gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
        gtsam::Pose3 wTc = optimizedValues_.at<gtsam::Pose3>(csym);
        for (auto const &tagkv: camkv.second) { // loop through all tags
          const auto &obsTag = tagkv.second;  // observed tag with image points
          std::map<int, Tag>::const_iterator ti = tags_.find(tagkv.first);
          if (ti == tags_.end()) {
            continue;
          }
          const Tag &tag = ti->second;
          if (ti->first != 3) {
            continue;
          }
          std::cout << "reproj: camid: " << camid << std::endl;
          c->print_intrinsics(std::cout);
          std::cout << "cam pose: " << wTc.inverse() << std::endl;
          std::cout << "sym: " << camid << " + " << frame_num << std::endl;
          std::cout << "same in green: " << std::endl;
          print_pose(std::cout, wTc.inverse());

          for (int i = 0; i < 4; i++) {   // loop over corners
            gtsam::Symbol wsym = getWSym(tag.id, i);
            gtsam::Point3 wX_opt = optimizedValues_.at<gtsam::Point3>(wsym);
            gtsam::Point2 ip(obsTag.corners[i].x, obsTag.corners[i].y);
            gtsam::Point3 wX = tag.getWorldCorner(i);
            gtsam::PinholeCamera<gtsam::Cal3DS2> camera(wTc, *camModel);
            gtsam::Point2 ipp  = camera.project(wX);
            gtsam::Point2 ippo = camera.project(wX_opt);
            of << frame_num << " " << camid << " " << tag.id << " " << i
               << " " << wX.x() << " " << wX.y() << " " << wX.z()
               << " " << ip.x() << " " << ip.y()
               << " " << ipp.x() << " " << ipp.y()
               << " " << ippo.x() << " " << ippo.y()
               << endl;
          }
        }
      }
    }
  }
  
  struct Err {
    double unrelaxed{0};
    double relaxed{0};
    int    count{0};
    void add (const Err &e) {
      unrelaxed += e.unrelaxed;
      relaxed += e.relaxed;
      count     += e.count;
    }
    void normalize() {
      double n = (count > 0) ? (1.0/(double)count) : 0;
      unrelaxed = std::sqrt(unrelaxed * n);
      relaxed = std::sqrt(relaxed * n);
    }
  };
  
  static std::vector<std::pair<int, Err>> sort_errors(const std::map<int, Err> &errMap) {
    auto cmp = [](std::pair<int,Err> const & a, std::pair<int,Err> const & b) {
      return ((a.second.unrelaxed != b.second.unrelaxed) ?
              (a.second.unrelaxed < b.second.unrelaxed) : (a.first < b.first));
    };
    std::vector<std::pair<int, Err>> tagsToErrPairs;
    for (auto const &a : errMap) {
      tagsToErrPairs.push_back(std::pair<int, Err>(a.first, a.second));
    }
    std::sort(tagsToErrPairs.begin(), tagsToErrPairs.end(), cmp);
    return (tagsToErrPairs);
  }
  
  void CalibTool::testReprojection(const std::string &filename) const {
    ROS_INFO_STREAM("writing reprojection diagnostics to file " << filename);
    std::ofstream of = open_file(filename);
    std::map<int, Err> tagidToErr;
    std::map<int, Err> camidToErr;
    Err totalError;
    for (auto const &framekv: obsTags_) { // loop over all frames
      int frame_num = framekv.first;
      for (auto const &camkv: framekv.second) { // loop over all cameras
        int camid = camkv.first;
        const CamPtr &c = cam_[camid];
        boost::shared_ptr<gtsam::Cal3DS2> camModel = makeCameraModel(c);
        gtsam::Symbol csym = getCamToWorldSym(camid, frame_num);
        gtsam::Pose3 wTc = optimizedValues_.at<gtsam::Pose3>(csym);
        for (auto const &tagkv: camkv.second) { // loop through all tags
          const auto &obsTag = tagkv.second;  // observed tag with image points
          std::map<int, Tag>::const_iterator ti = tags_.find(tagkv.first);
          if (ti == tags_.end()) {
            continue;
          }
          const Tag &tag = ti->second;
          Err err;
          for (int i = 0; i < 4; i++) {   // loop over corners
            gtsam::Symbol wsym = getWSym(tag.id, i);
            gtsam::Point3 wX_opt = optimizedValues_.at<gtsam::Point3>(wsym);
            gtsam::Point2 ip(obsTag.corners[i].x, obsTag.corners[i].y);
            gtsam::Point3 wX = tag.getWorldCorner(i);
            gtsam::PinholeCamera<gtsam::Cal3DS2> camera(wTc, *camModel);
            gtsam::Point2 ipp  = camera.project(wX);
            gtsam::Point2 ippo = camera.project(wX_opt);
            err.unrelaxed += (ipp.x()-ip.x())*(ipp.x()-ip.x()) + (ipp.y()-ip.y())*(ipp.y()-ip.y());
            err.relaxed += (ippo.x()-ip.x())*(ippo.x()-ip.x()) + (ippo.y()-ip.y())*(ippo.y()-ip.y());
          }
          err.count = 4;
          tagidToErr[tagkv.first].add(err);
          camidToErr[camid].add(err);
          totalError.add(err);
        }
      }
    }
    totalError.normalize();
    ROS_INFO_STREAM("reprojection error unrelaxed: " << totalError.unrelaxed
                    << " relaxed: " << totalError.relaxed);
    of << "reprojection error unrelaxed: " << totalError.unrelaxed
       << " relaxed: " << totalError.relaxed << endl;
    
    for (auto &p: tagidToErr) {  p.second.normalize(); }
    for (auto &p: camidToErr) {  p.second.normalize(); }

    auto tagsToErrPairs = sort_errors(tagidToErr);
    auto camsToErrPairs = sort_errors(camidToErr);
    of << "-------------------- error by tag ----------------" << endl;
    for (const auto &p: tagsToErrPairs) {
      of << "tag id: " << std::setw(3) << p.first << " unrelaxed err: " << std::setw(10) << p.second.unrelaxed << " relaxed: " << std::setw(10) << p.second.relaxed <<  endl;
    }
    of << "-------------------- error by camera ----------------" << endl;
    for (const auto &p: camsToErrPairs) {
      of << "cam id: " << std::setw(3) << p.first << " unrelaxed err: " << std::setw(10) << p.second.unrelaxed << " relaxed: " << std::setw(10) << p.second.relaxed <<  endl;
    }
  }

  bool CalibTool::addTag(const Tag &t) {
    if (tags_.find(t.id) != tags_.end()) {
      throw std::runtime_error("duplicate tag id: " + std::to_string(t.id));
    }
    if (tagSizeToNumber_.count(t.size) != 0) {
      tagSizeToNumber_[t.size] = tagSizeToNumber_.size();
    }
    Tag tag(t);
    // replace size number with right value
    tag.sizenumber = tagSizeToNumber_[t.size];

    tags_[t.id] = tag;

    return (true);
  }
}  // namespace

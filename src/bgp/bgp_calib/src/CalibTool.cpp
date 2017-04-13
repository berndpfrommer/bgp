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
#include <string>
#include <cmath>

using std::cout;
using std::endl;

namespace bgp_calib {
  static std::vector<gtsam::Point3> wps;
  static std::vector<std::pair<double, double>> ips;
  static int to_point_id(int tagid, int corner) {
    return (tagid * 4 + corner);
  }
  static char to_cam_sym(int id) {
    return ('a' + (uint8_t)id);
  }
  static gtsam::Symbol getWorldToCamSym(int camid, int frame) {
    return (gtsam::Symbol('c' + camid, frame));
  }
#if 0  
  static void print_trans(std::string name, const gtsam::Pose3 &p) {
    cout << name << "R = [";
    cout << p.rotation().matrix();
    cout << "];" << endl;
    cout << name << "T = ";
    cout << p.translation();
    cout << ";" << endl;
  }
#else
  static void print_trans(std::string name, const gtsam::Pose3 &p) {
    cout << name << " = [..." <<  endl;
    const auto H = p.matrix();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        cout << ", " << H(i,j);
      }
      if (i < 3) {
        cout << "; ..." << endl;
      } else {
        cout << "];" << endl;
      }
    }
  }
#endif
  
  static CalibTool::PoseNoise makePoseNoise(const Eigen::Vector3d &a,
                                            const Eigen::Vector3d &p) {
    gtsam::Vector sn(6);
    sn << a(0),a(1),a(2),p(0),p(1),p(2);
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }

  static gtsam::Matrix3 rotmat(const Eigen::Vector3d &rvec) {
    Eigen::Matrix<double, 3, 1> axis{0, 0, 0};
    double angle(0);
    const double n = rvec.norm();
    
    if (n > 1e-8) {
      axis = rvec / n;
      angle = n;
    }
    Eigen::Matrix<double, 3, 3> m(Eigen::AngleAxis<double>(angle, axis));
    return (gtsam::Matrix3(m));
  }

  static boost::shared_ptr<gtsam::Cal3DS2> makeCameraModel(CamPtr c) {
    const Eigen::Matrix<double, 3, 3> &K = c->K();
    const Eigen::Matrix<double, 1, 5> &D = c->D();
    boost::shared_ptr<gtsam::Cal3DS2>
      model(new gtsam::Cal3DS2(K(0,0),K(1,1),0, K(0,2),K(1,2),
                               D(0,0),D(0,1),D(0,2),D(0,3)));
    return (model);
  }

//#define DEBUG_PRINT

  static gtsam::Pose3 guessPose(CamPtr c, const CalibTool::Tag &tag,
                                const apriltag_msgs::Apriltag::_corners_type &corners)
  {
    
    std::vector<gtsam::Point2> ip;  // image points
    std::vector<gtsam::Point3> wp;  // image points
    for (int i = 0; i < 4; i++) {   // loop over corners
      gtsam::Point2 uv(corners[i].x, corners[i].y);
      ip.push_back(uv);
      wp.push_back(tag.getObjectCorner(i));
#ifdef DEBUG_PRINT
      cout << corners[i].x << " " << corners[i].y << " "
           << wp.back().x() << " " << wp.back().y() << " " << wp.back().z() << endl;
#endif
      gtsam::Point3 wp0(wp.back().x(), wp.back().y(), 0.0);
      gtsam::Point3 wpt = tag.pose.transform_from(wp0);

      wps.push_back(wpt);
      ips.push_back(std::pair<double,double>(corners[i].x, corners[i].y));
    }
#ifdef DEBUG_PRINT
    cout << "K: " << c->K() << endl;
#endif    

    gtsam::Pose3 guess = utils::get_init_pose(wp, ip, c->K());
    return (guess);
  }
  
  gtsam::Point3 CalibTool::Tag::getObjectCorner(int i) const {
    double s = size;
    const gtsam::Point3 c[4] = {gtsam::Point3(-s/2,-s/2, 0),
                                gtsam::Point3( s/2,-s/2, 0),
                                gtsam::Point3( s/2, s/2, 0),
                                gtsam::Point3(-s/2, s/2, 0)};
    return (c[i]);
  }

  gtsam::Point3 CalibTool::Tag::getWorldCorner(int i) const {
    return (pose.transform_from(getObjectCorner(i)));
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
  
  gtsam::Values CalibTool::optimize() {
#ifdef DEBUG_PRINT
    cout << "---------------- graph: ------------------- " << endl;
    graph_.print();
    cout << "---------------- values: ------------------- " << endl;
    values_.print();
    cout << "---------- output from optimizer ---------" << endl;
#endif    
    gtsam::LevenbergMarquardtParams lmp;
    lmp.setVerbosity("TERMINATION");
    //lmp.setVerbosity("ERROR");
    lmp.setMaxIterations(100);
    lmp.setAbsoluteErrorTol(1e-5);
    lmp.setRelativeErrorTol(0);
    gtsam::LevenbergMarquardtOptimizer lmo(graph_, values_, lmp);
    gtsam::Values result = lmo.optimize();
#ifdef DEBUG_PRINT
    cout << "------------------------------------------" << endl;
    result.print();
    cout << "-------------- SUCCESS -----------------" << endl;
#endif    
    printCameraPoses(result);
    return (result);
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

  void CalibTool::test() {
    gtsam::Symbol asym('a', 0);
    gtsam::Symbol csym('c', 0);
    double an(1e-6), posn(1e-6);
    PoseNoise pn = makePoseNoise(Eigen::Vector3d(an,an,an),
                                 Eigen::Vector3d(posn,posn,posn));
    
    const gtsam::Pose3 A(gtsam::Rot3::RzRyRx(0.1,0.2,0.3), gtsam::Point3( 1, 0, 0));
    const gtsam::Pose3 C(gtsam::Rot3::RzRyRx(0.4,1.2,0.3), gtsam::Point3(-1, 0, 0));

    graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(asym, A, pn));
    graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(csym, C, pn));
    values_.insert(asym, A);
    values_.insert(csym, C);
    // B = C * A^-1 or          C = BA
    //gtsam::Pose3 diff = C.compose(A.inverse()); // not working!
    //gtsam::Pose3 diff = C.compose(A); // not working ?
    //gtsam::Pose3 diff = C.inverse().compose(A); // not working
    //gtsam::Pose3 diff   = C.inverse().compose(A.inverse()); // not working
    
    //gtsam::Pose3 diff   = A.compose(C); // not working
    //gtsam::Pose3 diff   = A.compose(C.inverse());
    gtsam::Pose3 diff   = A.inverse().compose(C);   // works!!!!
    //gtsam::Pose3 diff     = A.inverse().compose(C.inverse());
    gtsam::NonlinearFactor::shared_ptr bf(
      new gtsam::BetweenFactor<gtsam::Pose3>(asym, csym, diff, pn));

    graph_.push_back(gtsam::AntiFactor(bf));

    optimize();
    exit(-1);
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
    gtsam::Symbol osym('o', tag.id);   // wTo 
    graph_.push_back(gtsam::PriorFactor<gtsam::Pose3>(osym, tag.pose, tag.noise));
    values_.insert(osym, tag.pose);
    
    auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4); // error in m!
    // now add world coordinate points "w" to grah and values
    for (int i = 0; i < 4; i++) {   // loop over corners
      // this will also insert the object coordinates X if needed!
      gtsam::Symbol xsym = getObjectCoordSym(tag, i);
      // add ref frame factor and corresponding values
      gtsam::Symbol wsym('w', 4 * tag.id + i);
      graph_.push_back(gtsam::ReferenceFrameFactor<gtsam::Point3,
                       gtsam::Pose3>(xsym, osym, wsym, noise));
      gtsam::Point3 oX = values_.at<gtsam::Point3>(xsym);
      values_.insert(wsym, tag.pose.transform_from(oX));
    }
  }
                                 
  
  void CalibTool::tagObserved(const ros::Time &t,
                              int camid, const apriltag_msgs::Apriltag &obstag) {
    if (camid < 0 || camid > cam_.size()) {
      ROS_ERROR("invalid cam id %d, total cams: %d", camid, (int)cam_.size());
      return;
    }
    if (tags_.count(obstag.id) == 0) {
      return;
    }
    ROS_INFO("using tag: %d", obstag.id);
    const Tag &tag = tags_[obstag.id];

    //graph_ = gtsam::NonlinearFactorGraph();
    //values_ = gtsam::Values();
    CamPtr &c = cam_[camid];
    int frame_num = c->getFrameNum(t);
    insertTagIfNew(frame_num, camid, obstag);

    cout << "frame num: " << frame_num << " camid: " << camid << endl;

    //
    // add projection factor
    //
    boost::shared_ptr<gtsam::Cal3DS2> camModel = makeCameraModel(c);
    IsotropicNoisePtr pixelNoise = gtsam::noiseModel::Isotropic::Sigma(2, 2.0);
    gtsam::Symbol csym = getWorldToCamSym(camid, frame_num);
    
    for (int i = 0; i < 4; i++) {   // loop over corners
      gtsam::Symbol wsym('w', 4 * tag.id + i);
      gtsam::Point2 uv(obstag.corners[i].x, obstag.corners[i].y);
      graph_.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3,
                       gtsam::Point3,
                       gtsam::Cal3DS2>(uv, pixelNoise, csym, wsym, camModel));
    }

    //
    // add guess for cTw = cTo * oTw
    //
    if (!values_.exists(csym)) {
      gtsam::Pose3 cTo = guessPose(c, tag, obstag.corners);
      print_trans("cTo", cTo);
      print_trans("wTo", tag.pose);
      gtsam::Pose3 wTc = tag.pose.compose(cTo.inverse());
      print_trans("wTc", wTc);
      values_.insert(csym, wTc);
    }

    //gtsam::Values result = optimize();
    //testReprojection(result, ip, camModel);
//#define CLEAR_IT
#ifdef CLEAR_IT
    values_.clear();
    graph_.erase(graph_.begin(), graph_.end());
#endif    
  }

  void CalibTool::testReprojection(const gtsam::Values &values,
                                   const std::vector<gtsam::Point2> &ips,
                                   boost::shared_ptr<gtsam::Cal3DS2> camModel) {
    std::vector<std::vector<double>> a;
    cout << "====================== reprojection test " << endl;
    for (auto const &framekv: obsTags_) {
      int frame_num = framekv.first;
      for (auto const &camkv: framekv.second) {
        int camid = camkv.first;
        gtsam::Symbol csym = getWorldToCamSym(camid, frame_num);
        gtsam::Pose3 cTw = values.at<gtsam::Pose3>(csym);
        print_trans("cTw", cTw);
        for (auto const &tagkv: camkv.second) {
          cout << " ---------- " << tagkv.first << " --------------" << endl;
          const auto &obsTag = tagkv.second;
          const Tag &tag = tags_[tagkv.first];
          gtsam::Symbol osym = getObjectToCamSym(camid, frame_num, tag.id);
          gtsam::Pose3 oTc = values.at<gtsam::Pose3>(osym);
          print_trans("wTo", tag.pose);
          print_trans("oTc", oTc);
          for (int i = 0; i < 4; i++) {   // loop over corners
            gtsam::Symbol xsym = getObjectCoordSym(tag, i);
            gtsam::Point3 oX = values.at<gtsam::Point3>(xsym); // object point
            gtsam::Point2 uv(obsTag.corners[i].x, obsTag.corners[i].y);
            gtsam::Point3 cX = oTc.inverse().transform_from(oX);
            gtsam::Point3 wX = tag.getWorldCorner(i);
            gtsam::PinholeCamera<gtsam::Cal3DS2> camera(cTw.inverse(), *camModel);
            gtsam::PinholeCamera<gtsam::Cal3DS2> cam0(gtsam::Pose3(), *camModel);
            gtsam::Point2 wpp = camera.project(wX);
            gtsam::Point2 cpp = cam0.project(cX);
            //
            //
            //
            a.push_back(std::vector<double>());
            auto &ab = a.back();
            //    1      2      3     4     5     6  7  8  9    10    11    12
            // fnum, camid, tagid, wX.x, wX.y, wX.z, u, v, wpu, wpv, cpu,  cpv
            ab.push_back(frame_num);
            ab.push_back(camid);
            ab.push_back(tag.id);
            ab.push_back(wX.x()); ab.push_back(wX.y()); ab.push_back(wX.z());
            ab.push_back(uv.x()); ab.push_back(uv.y());
            ab.push_back(wpp.x()); ab.push_back(wpp.y());
            ab.push_back(cpp.x()); ab.push_back(cpp.y());
          }
        }
      }
    }

    cout << "============== points =============" << endl;
    cout << "a=[" << endl;
    for (const auto& l: a) {
      for (const auto& e: l) {
        cout << " " << e;
      }
      cout << endl;
    }
    cout << "]" << endl;
  }

  bool CalibTool::addTag(int id, double size,
                         const Eigen::Vector3d &anglevec,
                         const Eigen::Vector3d &center,
                         const Eigen::Vector3d &angnoise,
                         const Eigen::Vector3d &posnoise) {
    if (tags_.find(id) != tags_.end()) {
      throw std::runtime_error("duplicate tag id: " + std::to_string(id));
    }
    gtsam::Rot3   R(rotmat(anglevec));
    gtsam::Point3 T(center);
    gtsam::Pose3  trans(R, T);
    if (tagSizeToNumber_.count(size) != 0) {
      tagSizeToNumber_[size] = tagSizeToNumber_.size();
    }

    Tag t(id, tagSizeToNumber_[size], size,
          trans, makePoseNoise(angnoise, posnoise));
    tags_[id] = t;
#ifdef TEST_JUNK
    // ------------ test code --------------
    apriltag_msgs::Apriltag ttag;
    ttag.id = id;
    Eigen::Vector3d rv(0.0, 0.0, M_PI * 0.25);
    gtsam::Rot3   fR(rotmat(rv));
    gtsam::Pose3 ft(fR, gtsam::Point3(0, 0, 0));
    cout << "transform: " << ft << endl;
    for (int i = 0; i < 4; i++) {
      gtsam::Point3 p = t.getObjectCorner(i);
      gtsam::Point3 tp = ft.transform_to(p);
      cout << "transformed point: " << tp << endl;
      ttag.corners[i].x = tp.x();
      ttag.corners[i].y = tp.y();
    }
    tagObserved(ros::Time::now(), 0, ttag);
    // ---------- end test code --------------
    exit(-1);
#endif    

    return (true);
  }
}  // namespace

/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef BGPCALIB_CALIBTOOL_H
#define BGPCALIB_CALIBTOOL_H

#include <bgp_calib/camera.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ReferenceFrameFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <ros/ros.h>
#include <apriltag_msgs/Apriltag.h>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <Eigen/Dense>

namespace bgp_calib {
  class CalibTool {
  public:
    typedef gtsam::noiseModel::Diagonal::shared_ptr   PoseNoise;
    struct Tag {
      Tag(int ida = 0, int num = 0, double sz = 0,
          const gtsam::Pose3 &p = gtsam::Pose3(),
          PoseNoise pn = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector(6))):
        id(ida), sizenumber(num), size(sz), pose(p), noise(pn) { };
      gtsam::Point3 getObjectCorner(int i) const;
      gtsam::Point3 getWorldCorner(int i) const;
      int           id;
      int           sizenumber; // counts tag sizes
      double        size;
      gtsam::Pose3  pose; // transforms object to world
      PoseNoise     noise;
    };

    CalibTool() {};
    ~CalibTool() {};
    CalibTool(const CalibTool&) = delete;
    CalibTool& operator=(const CalibTool&) = delete;

    void tagObserved(const ros::Time &t, int camid,
                     const apriltag_msgs::Apriltag &obstag);
    bool addTag(int id, double size, const Eigen::Vector3d &anglevec,
                const Eigen::Vector3d &center,
                const Eigen::Vector3d &angnoise,
                const Eigen::Vector3d &posnoise);
    void addCamera(CamPtr cam) { cam_.push_back(cam); }
    void test();

  private:
    gtsam::Values  optimize();
    void  testReprojection(const gtsam::Values &values,
                           const std::vector<gtsam::Point2> &ips,
                           boost::shared_ptr<gtsam::Cal3DS2> camModel);
    typedef gtsam::noiseModel::Isotropic::shared_ptr	IsotropicNoisePtr;
    typedef gtsam::noiseModel::Diagonal::shared_ptr		DiagonalNoisePtr;
    typedef gtsam::ReferenceFrameFactor<gtsam::Point3, gtsam::Pose3> RefFrameFactor;
    
    std::vector<gtsam::Point3> getTagCorners(double s);
    int           tagIdToTagNumber(int tagid) const;
    gtsam::Symbol getObjectToCamSym(int camid, int frame, int tagid);
    gtsam::Symbol getObjectCoordSym(const Tag &tag, int corner);
    void insertTagIfNew(int frame_num, int camid,
                        const apriltag_msgs::Apriltag &otag);
    // -------------------------------------------------------------
    typedef std::map<int, apriltag_msgs::Apriltag> TagsMap;
    typedef std::map<int, TagsMap>        CamToTagsMap;
    
    std::map<int, CamToTagsMap> obsTags_; // observed tags for each frame
    gtsam::Values                 values_;
    gtsam::NonlinearFactorGraph   graph_;
    std::map<int,    Tag>         tags_;
    std::map<double, int>         tagSizeToNumber_;
    std::vector<CamPtr>           cam_;
  };

}

#endif

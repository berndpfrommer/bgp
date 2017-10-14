/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef BGPCALIB_CALIBTOOL_H
#define BGPCALIB_CALIBTOOL_H

#include <bgp_calib/camera.h>
#include <bgp_calib/tag.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ReferenceFrameFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

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
    CalibTool() {};
    ~CalibTool() {};
    CalibTool(const CalibTool&) = delete;
    CalibTool& operator=(const CalibTool&) = delete;

    void setMaxError(double e) { maxError_ = e; }
    void addCamera(CamPtr cam) { cam_.push_back(cam); }
    bool addTag(const Tag &t);
    int  frameObserved(const apriltag_msgs::ApriltagArrayStamped::ConstPtr &msg,
                      int camid);
    gtsam::Values  optimize();
    void clear();
 
    void printResults() const;
    bool allCamerasHaveFrames() const;
    bool gotFrames(int camid) const { return (cam_[camid]->gotFrames()); }
    void writeCalibrationFile(const std::string &filename) const;
    void writeCameraPoses(const std::string &filename) const;
    void writeTagPoses(const std::string &filename) const;
    void writeReprojectionData(const std::string &filename) const;
    void testReprojection(const std::string &filename) const;
  private:
    typedef boost::random::mt19937 RandEng;
    typedef boost::random::normal_distribution<double> RandDist;
    typedef boost::random::variate_generator<RandEng, RandDist> RandGen;

    bool tagObserved(int frame_num, int camid,
                     const apriltag_msgs::Apriltag &obstag);

    typedef gtsam::noiseModel::Isotropic::shared_ptr	IsotropicNoisePtr;
    typedef gtsam::noiseModel::Diagonal::shared_ptr		DiagonalNoisePtr;
    typedef gtsam::ReferenceFrameFactor<gtsam::Point3, gtsam::Pose3> RefFrameFactor;
    
    std::vector<gtsam::Point3> getTagCorners(double s);
    int           tagIdToTagNumber(int tagid) const;
    gtsam::Symbol getObjectToCamSym(int camid, int frame, int tagid);
    gtsam::Symbol getObjectCoordSym(const Tag &tag, int corner);
    void insertTagIfNew(int frame_num, int camid,
                        const apriltag_msgs::Apriltag &otag);
    gtsam::Pose3 makeRandomPose(RandGen *rgr, RandGen *rgt);
    bool makeCameraPositionGuess(int frame_num, int camid);
    void updateStartingPose(int frame_num, int camid, const gtsam::Pose3 &wTc);
    gtsam::Pose3 guessPoseFromHomographies(int frame_num, int camid);
    // -------------------------------------------------------------
    typedef std::map<int, apriltag_msgs::Apriltag> TagsMap;
    typedef std::map<int, TagsMap>        CamToTagsMap;
    
    std::map<int, CamToTagsMap>   obsTags_; // observed tags for each frame
    gtsam::Values                 values_;
    gtsam::Values                 optimizedValues_;
    gtsam::NonlinearFactorGraph   graph_;
    std::map<int,    Tag>         tags_;
    std::map<double, int>         tagSizeToNumber_;
    std::vector<CamPtr>           cam_;
    double                        maxError_{200};
  };

}

#endif

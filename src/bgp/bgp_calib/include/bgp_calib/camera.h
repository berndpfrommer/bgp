/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef BGP_CALIB_CAMERA_H
#define BGP_CALIB_CAMERA_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <memory>
#include <map>

namespace bgp_calib {
  class Camera {
  public:
    Camera(const std::string &name = "") : name_(name) {}

    void setK(const std::vector<double> &intrinsics);
    void setD(const std::vector<double> &coeff);
    void setP(const Eigen::Matrix<double, 3,3> &P);
    void setR(const Eigen::Matrix<double, 3,3> &R);
    void setTransform(const std::vector<double> &transf);
    void setRes(const std::vector<int> &res);
    void setDistortionModel(const std::string &distmodel);

    const std::string &getName() const { return (name_); }
    const Eigen::Matrix<double,3,3>  &K() const {
      return (K_); } // intrinsic calib
    const Eigen::Matrix<double, 1, 5> &D() const {
      return (D_); } // distortion coefficients
    const Eigen::Matrix<double, 3, 3>  &P() const {
      return (P_); } // post-rectify project matrix
    const Eigen::Matrix<double, 3, 3>  &R() const {
      return (R_); } // stereo rect rot mat
    Eigen::Matrix<double, 3, 3> extrinsicR() const; // extrinsic rot matrix
    Eigen::Matrix<double, 3, 1> extrinsicT() const; // extrinsic trans vector
    const int *res() const { return (res_); }

    int getFrameNum(const ros::Time &t);
    void initialize(ros::NodeHandle *nh,  const std::string &camName);

  private:
    std::string                 name_;
    std::string                 distModel_;
    Eigen::Matrix<double, 3, 3> K_;       // intrinsic matrix [3x3]
    Eigen::Matrix<double, 1, 5> D_;       // distortion coefficients [1 x n]
    Eigen::Matrix<double, 4, 4> T_;       // extrinsic (pose) [4 x 4];
    Eigen::Matrix<double, 3, 3> R_;       // rectification rotation matrix [3x3]
    Eigen::Matrix<double, 3, 3> P_;       // new projection (K) matrix after rect
    int                         res_[2];  // image resolution
    sensor_msgs::CameraInfo     camInfo_;
    ros::Time                   lastTime_;
    int                         frameNum_{-1};
  };
  typedef std::shared_ptr<Camera> CamPtr;
}

#endif

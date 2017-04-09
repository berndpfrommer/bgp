/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <bgp_calib/camera.h>
#include <ros/console.h>
#include <string>
#include <vector>
#include <ctime>

using std::vector;
using std::string;

namespace bgp_calib {
  void Camera::setDistortionModel(const std::string &distmodel) {
    distModel_ = distmodel;
    camInfo_.distortion_model = distmodel;
  }
  void Camera::setK(const std::vector<double> &k) {
    K_ << k[0], 0, k[2], 0, k[1], k[3], 0.0, 0.0, 1.0;
    camInfo_.K = {k[0], 0, k[2],   0, k[1], k[3],    0, 0, 1.0};
  }
  void Camera::setR(const Eigen::Matrix3d &R) {
    R_ = R;
    camInfo_.R = {R(0,0), R(0,1), R(0,2),
                  R(1,0), R(1,1), R(1,2),
                  R(2,0), R(2,1), R(2,2)};
  }
  void Camera::setP(const Eigen::Matrix3d &P) {
    P_ = P;
    camInfo_.P = {P(0,0), P(0,1), P(0,2),
                  P(1,0), P(1,1), P(1,2),
                  P(2,0), P(2,1), P(2,2)};
  }
  void Camera::setD(const std::vector<double> &dc) {
    D_ = Eigen::Matrix<double, 1, 5>::Zero();
    camInfo_.D = std::vector<double>(5, 0);
    for (int i = 0; i < dc.size(); i++) {
      D_(0, i) = dc[i];
      camInfo_.D[i] = dc[i];
    }
  }
  void Camera::setTransform(const std::vector<double> &T) {
    // assumes input as 4x4 matrix
    if (T.size() == 16) {
      T_ = Eigen::Matrix<double, 4, 4>();
      for (int i = 0; i < T.size(); i++) {
        T_(i/4, i%4) = T[i];
      }
    }
  }
  void Camera::setRes(const std::vector<int> &res) {
    if (res.size() == 2) {
      res_[0] = res[0];
      res_[1] = res[1];
    }
    camInfo_.width  = res[0];
    camInfo_.height = res[1];
  }
  
  Eigen::Matrix<double, 3, 3> Camera::extrinsicR() const {
    return (T_.block<3,3>(0, 0));
  }

  Eigen::Matrix<double, 3, 1> Camera::extrinsicT() const {
    return (T_.block<3,1>(0, 3));
  }

  int Camera::getFrameNum(const ros::Time &t) {
    if (fabs((t - lastTime_).toSec()) > 1e-4) {
      frameNum_++;
      lastTime_ = t;
    }
    return (frameNum_);
  }

  template <typename T>
  static T get_param(ros::NodeHandle *nh, std::string name, const T &def) {
    T val;
    if (!nh->param(name, val, def)) {
      throw (std::runtime_error("cannot find parameter " + name));
    }
    return (val);
  }
  
  void Camera::initialize(ros::NodeHandle *nh,
                          const std::string &camName) {
    std::string topic = camName + "_output_rect";
    std::string info_topic = camName + "_output_caminfo";
    setK(get_param(nh, camName + "/intrinsics",  vector<double>()));
    setD(get_param(nh, camName + "/distortion_coeffs", vector<double>()));
    setDistortionModel(get_param(nh, camName + "/distortion_model",
                                 string("equidistant")));
    setTransform(get_param(nh, camName + "/T_cam_imu", vector<double>()));
    setRes(get_param(nh, camName + "/resolution", vector<int>()));
  }
}

/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef BGPCALIB_UTILS_H
#define BGPCALIB_UTILS_H

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Dense>
#include <vector>

namespace bgp_calib {
  // computes a pose (rotation vector, translation) via homography
  // from world and image points. Distortion is not taken into account,
  // obviously this is just a starting guess.
  namespace utils {
    gtsam::Pose3 get_init_pose(const std::vector<gtsam::Point3> &world_points,
                               const std::vector<gtsam::Point2> &image_points,
                               const Eigen::Matrix<double, 3, 3> &K);
  }
}

#endif

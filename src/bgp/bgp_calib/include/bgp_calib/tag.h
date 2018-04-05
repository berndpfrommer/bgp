/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */
#ifndef BGP_CALIB_TAG_H
#define BGP_CALIB_TAG_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Pose3.h>
#include <ros/ros.h>
#include <vector>

namespace bgp_calib {
  struct Tag {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef gtsam::noiseModel::Diagonal::shared_ptr   PoseNoise;
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
    static std::vector<Tag> parseTags(XmlRpc::XmlRpcValue poses);
  };
}

#endif

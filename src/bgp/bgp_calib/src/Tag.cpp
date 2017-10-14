/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <bgp_calib/tag.h>
#include <ros/ros.h>

using std::cout;
using std::endl;

namespace bgp_calib {
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
  static Tag::PoseNoise make_pose_noise(const Eigen::Vector3d &a,
                                        const Eigen::Vector3d &p) {
    gtsam::Vector sn(6);
    sn << a(0),a(1),a(2),p(0),p(1),p(2);
    return (gtsam::noiseModel::Diagonal::Sigmas(sn));
  }
  static Eigen::Vector3d get_vec(XmlRpc::XmlRpcValue v) {
    double x(0), y(0), z(0);
    for (XmlRpc::XmlRpcValue::iterator it = v.begin();
         it != v.end(); ++it) {
      std::string field = it->first;
      if (field == "x") {        x = static_cast<double>(it->second);
      } else if (field == "y") { y = static_cast<double>(it->second);
      } else if (field == "z") { z = static_cast<double>(it->second);
      }
    }
    return (Eigen::Vector3d(x, y, z));
  }

  gtsam::Point3 Tag::getObjectCorner(int i) const {
    double s = size;
    const gtsam::Point3 c[4] = {gtsam::Point3(-s/2,-s/2, 0),
                                gtsam::Point3( s/2,-s/2, 0),
                                gtsam::Point3( s/2, s/2, 0),
                                gtsam::Point3(-s/2, s/2, 0)};
    return (c[i]);
  }

  gtsam::Point3 Tag::getWorldCorner(int i) const {
    return (pose.transform_from(getObjectCorner(i)));
  }
  
  std::vector<Tag> Tag::parseTags(XmlRpc::XmlRpcValue poses) {
    std::vector<Tag> tags;
    for (uint32_t i = 0; i < poses.size(); i++) {
      if (poses[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;
      int id(0);
      double sz(0.1), uc(0);
      Eigen::Vector3d anglevec, center, rotnoise, posnoise;

      for (XmlRpc::XmlRpcValue::iterator it = poses[i].begin();
           it != poses[i].end(); ++it) {
        std::string field = it->first;
        if (field == "id") {           id = static_cast<int>(it->second);
          //ROS_INFO("found tag with id %d", id);
        } else  if (field == "size") { sz = static_cast<double>(it->second);
        } else  if (field == "uncertainty") {
          uc = static_cast<double>(it->second);
        } else if (field == "rotvec") { anglevec = get_vec(it->second);
        } else if (field == "center") { center   = get_vec(it->second);
        } else if (field == "rotation_noise") { rotnoise = get_vec(it->second);
        } else if (field == "position_noise") { posnoise = get_vec(it->second);
        }
      }
      gtsam::Rot3   R(rotmat(anglevec));
      gtsam::Point3 T(center);
      gtsam::Pose3  trans(R, T);
      Tag t(id, 0 /*num*/, sz,   trans, make_pose_noise(rotnoise, posnoise));

      tags.push_back(t);
    }
    return (tags);
  }
}  // namespace

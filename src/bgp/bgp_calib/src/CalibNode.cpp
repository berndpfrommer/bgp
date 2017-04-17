/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <bgp_calib/calib_node.h>

using std::stoi;
using std::stod;
namespace bgp_calib {
  CalibNode::CalibNode(const ros::NodeHandle& pnh) :
    nh_(pnh)
  {
  }

  CalibNode::~CalibNode() {
  }

  void CalibNode::tag_cb(const
                         apriltag_msgs::ApriltagArrayStamped::ConstPtr &msg,
                         int camid) {
    // if this is first frame for camera, process it
    if (!calibTool_.gotFrames(camid)) {
      int numUsed(0);
      for (const auto &tag: msg->apriltags) {
        bool used = calibTool_.tagObserved(msg->header.stamp, camid, tag);
        if (used) numUsed++;
      }
      std::cout << "cam " << camid << " processed " << numUsed << " of " <<
        msg->apriltags.size() << " tags!" << std::endl;
    }
    if (calibTool_.allCamerasHaveFrames()) {
      calibTool_.optimize();
      calibTool_.printResults();
      std::string calibFileName("calib.yaml");
      nh_.getParam("calib_file_name", calibFileName);
      calibTool_.writeCalibrationFile(calibFileName);
      ros::shutdown();
    }
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

  bool CalibNode::initialize() {
    std::vector<std::string> cams;
    if (!nh_.getParam("camera_names", cams)) {
      throw std::runtime_error("no camera names specified!!!");
    }

    for (int id = 0; id < cams.size(); id++) {
      const std::string &cname = cams[id];
      CamPtr cam(new Camera(cname, id, this));
      cam->initialize(&nh_, cname);
      calibTool_.addCamera(cam);
      ROS_INFO("loaded camera calibration for %s", cname.c_str());
    }

    XmlRpc::XmlRpcValue poses;
    nh_.getParam("tag_poses", poses);

    // iterate through all poses
    for (uint32_t i = 0; i < poses.size(); i++) {
      if (poses[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) continue;

      int id(0);
      double sz(0.1), uc(0);
      Eigen::Vector3d anglevec, center, rotnoise, posnoise;

      for (XmlRpc::XmlRpcValue::iterator it = poses[i].begin();
           it != poses[i].end(); ++it) {
        std::string field = it->first;
        if (field == "id") {           id = static_cast<int>(it->second);
          ROS_INFO("found tag with id %d", id);
        } else  if (field == "size") { sz = static_cast<double>(it->second);
        } else  if (field == "uncertainty") {
          uc = static_cast<double>(it->second);
        } else if (field == "rotvec") { anglevec = get_vec(it->second);
        } else if (field == "center") { center   = get_vec(it->second);
        } else if (field == "rotation_noise") { rotnoise = get_vec(it->second);
        } else if (field == "position_noise") { posnoise = get_vec(it->second);
        }
      }
      calibTool_.addTag(id, sz, anglevec, center, rotnoise, posnoise);
    }
    ROS_INFO("found poses for %d tags", poses.size());
    return (true);
  }
}  // namespace

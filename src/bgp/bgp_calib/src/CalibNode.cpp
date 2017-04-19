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
      calibTool_.frameObserved(msg, camid);
    }
    if (calibTool_.allCamerasHaveFrames()) {
      calibTool_.optimize();
      calibTool_.printResults();
      std::string calibFileName("calib.yaml"), tagPoses("tag_poses.txt"),
        camPoses("cam_poses.txt"), reprojData("reproj.txt"),
        reprojDiag("reproj_diag.txt");
      nh_.getParam("calib_file_name", calibFileName);
      nh_.getParam("output_tag_poses_file", tagPoses);
      nh_.getParam("output_cam_poses_file", camPoses);
      nh_.getParam("diagnostics_file", reprojDiag);
      nh_.getParam("reprojection_file", reprojData);
      calibTool_.writeCalibrationFile(calibFileName);
      calibTool_.writeTagPoses(tagPoses);
      calibTool_.writeCameraPoses(camPoses);
      calibTool_.writeReprojectionData(reprojData);
      calibTool_.testReprojection(reprojDiag);
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
    double maxError(200.0);
    nh_.getParam("max_error", maxError);
    calibTool_.setMaxError(maxError);

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
      calibTool_.addTag(id, sz, anglevec, center, rotnoise, posnoise);
    }
    ROS_INFO("loaded poses for %d tags", poses.size());
    return (true);
  }
}  // namespace

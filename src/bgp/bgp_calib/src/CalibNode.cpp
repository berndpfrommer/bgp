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

    std::vector<Tag> tags = Tag::parseTags(poses);
    for (const Tag &t: tags) {
      calibTool_.addTag(t);
    }
    ROS_INFO("loaded poses for %d tags", poses.size());
    return (true);
  }
}  // namespace

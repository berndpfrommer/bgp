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
                         apriltag_msgs::ApriltagArrayStamped::ConstPtr &msg) {
    
#if 1
    const auto &tags = msg->apriltags;
    for (const auto &tag: tags) {
      calibTool_.tagObserved(msg->header.stamp, 0, tag);
    }
    calibTool_.optimize();
#else
    const int mytag = 42;
    const auto &tags = msg->apriltags;
    for (const auto &tag: tags) {
      //std::cout << "got tag: " << tag.id << std::endl;
      //calibTool_.tagObserved(msg->header.stamp, 0, tag);
      if (tag.id == mytag) {
        std::cout << "FOUND TAG " << mytag << std::endl;
        for (int i= 0; i < 4; i++) {
          std::cout << "tag.corners["<<i<<"].x = " << tag.corners[i].x << ";" << std::endl;
          std::cout << "tag.corners["<<i<<"].y = " << tag.corners[i].y << ";" << std::endl;
        }
      }
    }
    apriltag_msgs::Apriltag tag;
    tag.id = mytag;
    tag.corners[0].x = 1552.51;
    tag.corners[0].y = 782.955;
    tag.corners[1].x = 1637.1;
    tag.corners[1].y = 786.413;
    tag.corners[2].x = 1636.81;
    tag.corners[2].y = 699.18;
    tag.corners[3].x = 1551.9;
    tag.corners[3].y = 696.373;
    
    calibTool_.tagObserved(msg->header.stamp, 0, tag);
#endif
    exit(-1);
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

    for (const std::string &cname: cams) {
      CamPtr cam(new Camera(cname));
      cam->initialize(&nh_, cname);
      calibTool_.addCamera(cam);
      ROS_INFO("loaded camera calibration for %s", cname.c_str());
    }
    sub_ = nh_.subscribe("apriltags", 1, &CalibNode::tag_cb, this);

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

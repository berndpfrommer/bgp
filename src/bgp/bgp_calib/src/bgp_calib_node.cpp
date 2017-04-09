/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#include <ros/ros.h>
#include <bgp_calib/calib_node.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bgp_calib_node");
  ros::NodeHandle pnh("~");

  try {
    bgp_calib::CalibNode node(pnh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", pnh.getNamespace().c_str(), e.what());
  }
}

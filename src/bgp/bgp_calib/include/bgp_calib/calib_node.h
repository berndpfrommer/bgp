/* -*-c++-*--------------------------------------------------------------------
 * 2016 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#ifndef BGPCALIB_CALIBNODE_H
#define BGPCALIB_CALIBNODE_H

#include <ros/ros.h>
#include <apriltag_msgs/ApriltagArrayStamped.h>
#include <bgp_calib/calib_tool.h>
#include <vector>

namespace bgp_calib {
  class CalibNode {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CalibNode(const ros::NodeHandle &pnh);
    ~CalibNode();

    CalibNode(const CalibNode&) = delete;
    CalibNode& operator=(const CalibNode&) = delete;

    bool initialize();
    void tag_cb(const apriltag_msgs::ApriltagArrayStamped::ConstPtr &msg, int camid);
 
  private:
    ros::NodeHandle     nh_;
    CalibTool           calibTool_;
  };

}

#endif

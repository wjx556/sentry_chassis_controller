#pragma once

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace sentry_chassis_controller {

class SimpleController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  std::vector<hardware_interface::JointHandle> vel_joints_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_br_;
  std::vector<control_toolbox::Pid> pids_;
  geometry_msgs::Twist cmd_vel_;
  nav_msgs::Odometry odom_;
};

}  // namespace sentry_chassis_controller

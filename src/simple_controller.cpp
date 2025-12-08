#include <sentry_chassis_controller/simple_controller.h>

namespace sentry_chassis_controller {

bool SimpleController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh)
{
  // 获取关节句柄（示例：4 轮 + 4 舵 = 8 关节）
  const std::vector<std::string> joint_names = {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
                                                "steer_fl", "steer_fr", "steer_rl", "steer_rr"};
  for (const auto& name : joint_names) {
    vel_joints_.push_back(hw->getHandle(name));
  }

  // 加载 PID 参数
  pids_.resize(vel_joints_.size());
  for (size_t i = 0; i < vel_joints_.size(); ++i) {
    ros::NodeHandle pid_nh(nh, "gains/" + joint_names[i]);
    pids_[i].init(pid_nh, false);
  }

  cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, [&](const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_vel_ = *msg;
  });
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  return true;
}

void SimpleController::update(const ros::Time& time, const ros::Duration& period)
{
  // 1. 逆运动学：Twist -> 8 个关节速度
  // 2. PID 计算
  // 3. 正运动学 -> 发布 odom & tf
  // 以下仅占位，后续再实现
  for (size_t i = 0; i < vel_joints_.size(); ++i) {
    double desired = 0.0;  // TODO：逆运动学结果
    double output = pids_[i].computeCommand(desired - vel_joints_[i].getVelocity(), period);
    vel_joints_[i].setCommand(output);
  }
}

void SimpleController::starting(const ros::Time& /*time*/) {}
void SimpleController::stopping(const ros::Time& /*time*/) {}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SimpleController,
                       controller_interface::ControllerBase)

#pragma once

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/SentryPidsConfig.h>

namespace sentry_chassis_controller {

    // 一个舵轮模组的结构体
    struct SwerveModule {
        std::string name;
        hardware_interface::JointHandle drive_joint;
        hardware_interface::JointHandle pivot_joint;
        control_toolbox::Pid drive_pid;
        control_toolbox::Pid pivot_pid;
        double x_offset; // 相对于base_link的x坐标
        double y_offset; // 相对于base_link的y坐标
    };

    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        SentryChassisController();
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
        void update(const ros::Time& time, const ros::Duration& period) override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;

    private:
        // 回调函数
        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        
        // 核心算法
        void updateOdometry(const ros::Time& time, const ros::Duration& period);
        void computeInverseKinematics(double vx, double vy, double omega, const ros::Duration& period);
        
        // 成员变量
        std::vector<SwerveModule> modules_;
        ros::Subscriber cmd_vel_sub_;
        
        // 实时发布器
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
        
        // 数据缓存
        geometry_msgs::Twist command_struct_;
        std::mutex cmd_mutex_;
        
        // TF 相关
        tf2_ros::TransformBroadcaster tf_broadcaster_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        
        // 里程计状态
        double odom_x_ = 0.0;
        double odom_y_ = 0.0;
        double odom_theta_ = 0.0;
        
        // 参数
        std::string odom_frame_id_;
        std::string base_frame_id_;
        bool enable_odom_tf_; // 题7：是否发布TF
        bool use_world_frame_control_; // 题7：世界坐标系控制模式
        double wheel_radius_;
        double max_power_; // 题8：功率限制模拟
        bool spinning_mode_; // 题8：小陀螺模式

        //Dynamic Reconfigure Server 和 Callback
        std::shared_ptr<dynamic_reconfigure::Server<sentry_chassis_controller::SentryPidsConfig>> dyn_server_;
        void reconfigCallback(sentry_chassis_controller::SentryPidsConfig& config, uint32_t level);
    };
    };

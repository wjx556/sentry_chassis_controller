#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace sentry_chassis_controller {

    SentryChassisController::SentryChassisController() : spinning_mode_(false) {}

    bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
        // 1. 读取参数
        controller_nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        controller_nh.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        controller_nh.param<bool>("enable_odom_tf", enable_odom_tf_, true);
        controller_nh.param<bool>("use_world_frame_control", use_world_frame_control_, false);
        controller_nh.param<double>("wheel_radius", wheel_radius_, 0.075);
        controller_nh.param<double>("max_power", max_power_, 100.0);

        // 2. 初始化 TF Listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // 3. 获取关节句柄并初始化 PID
        std::vector<std::string> module_names;
        if (!controller_nh.getParam("module_names", module_names)) {
            ROS_ERROR("No module names parameters");
            return false;
        }

        for (const auto& name : module_names) {
            SwerveModule module;
            module.name = name;
            
            // 获取位置参数
            if (!controller_nh.getParam(name + "/x_offset", module.x_offset) || 
                !controller_nh.getParam(name + "/y_offset", module.y_offset)) {
                ROS_ERROR_STREAM("Missing offset params for " << name);
                return false;
            }

            // 获取关节句柄
            std::string drive_joint_name, pivot_joint_name;
            controller_nh.param(name + "/drive_joint", drive_joint_name, name + "_drive_joint");
            controller_nh.param(name + "/pivot_joint", pivot_joint_name, name + "_pivot_joint");
            
            module.drive_joint = hw->getHandle(drive_joint_name);
            module.pivot_joint = hw->getHandle(pivot_joint_name);

            // 初始化 PID
            if (!module.drive_pid.init(ros::NodeHandle(controller_nh, name + "/drive_pid")) ||
                !module.pivot_pid.init(ros::NodeHandle(controller_nh, name + "/pivot_pid"))) {
                ROS_ERROR("Failed to init PID");
                return false;
            }

            modules_.push_back(module);
        }

        // 4. 初始化发布者和订阅者
        odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
        cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &SentryChassisController::cmdVelCallback, this);

        // 5. 初始化 Dynamic Reconfigure Server (新增部分)
        ros::NodeHandle nh_priv(controller_nh, "pid_reconfigure");
        dyn_server_ = std::make_shared<dynamic_reconfigure::Server<sentry_chassis_controller::SentryPidsConfig>>(nh_priv);
        dyn_server_->setCallback(boost::bind(&SentryChassisController::reconfigCallback, this, _1, _2));

        return true;
    }

    void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
        // 获取指令
        geometry_msgs::Twist cmd;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            cmd = command_struct_;
        }

        // 题7: 世界坐标系下的速度控制
        if (use_world_frame_control_) {
            try {
                geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(base_frame_id_, odom_frame_id_, ros::Time(0));
                
                double yaw = 2 * atan2(transform.transform.rotation.z, transform.transform.rotation.w); 
                double vx_new = cmd.linear.x * cos(yaw) + cmd.linear.y * sin(yaw);
                double vy_new = -cmd.linear.x * sin(yaw) + cmd.linear.y * cos(yaw);
                cmd.linear.x = vx_new;
                cmd.linear.y = vy_new;
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "TF Error: %s", ex.what());
            }
        }

        // 题8: 特色功能 - 小陀螺
        if (spinning_mode_) {
            cmd.angular.z += 2.0; 
        }

        computeInverseKinematics(cmd.linear.x, cmd.linear.y, cmd.angular.z, period);
        updateOdometry(time, period);
    }

    void SentryChassisController::computeInverseKinematics(double vx, double vy, double omega, const ros::Duration& period) {
        for (auto& module : modules_) {
            double vx_wheel = vx - omega * module.y_offset;
            double vy_wheel = vy + omega * module.x_offset;

            double target_speed = std::hypot(vx_wheel, vy_wheel);
            double target_angle = std::atan2(vy_wheel, vx_wheel);

            double current_angle = module.pivot_joint.getPosition();
            
            double angle_diff = angles::shortest_angular_distance(current_angle, target_angle);
            
            if (fabs(angle_diff) > M_PI / 2.0) {
                target_angle = angles::normalize_angle(target_angle + M_PI);
                target_speed = -target_speed;
            }
            
            if (std::abs(target_speed) > max_power_) {
                target_speed = (target_speed > 0 ? 1 : -1) * max_power_;
            }

            if (fabs(vx) < 1e-3 && fabs(vy) < 1e-3 && fabs(omega) < 1e-3) {
                 target_speed = 0.0;
                 target_angle = atan2(module.y_offset, module.x_offset); 
            }

            double pivot_error = angles::shortest_angular_distance(current_angle, target_angle);
            double pivot_effort = module.pivot_pid.computeCommand(pivot_error, period);
            
            double drive_error = (target_speed / wheel_radius_) - module.drive_joint.getVelocity();
            double drive_effort = module.drive_pid.computeCommand(drive_error, period);

            module.pivot_joint.setCommand(pivot_effort);
            module.drive_joint.setCommand(drive_effort);
        }
    }

    void SentryChassisController::updateOdometry(const ros::Time& time, const ros::Duration& period) {
        double sum_vx = 0, sum_vy = 0, sum_omega = 0;
        
        for (const auto& module : modules_) {
            double v_wheel = module.drive_joint.getVelocity() * wheel_radius_;
            double theta_pivot = module.pivot_joint.getPosition();
            
            double vx_i = v_wheel * cos(theta_pivot);
            double vy_i = v_wheel * sin(theta_pivot);
            
            sum_vx += vx_i;
            sum_vy += vy_i;
            
            double r2 = module.x_offset*module.x_offset + module.y_offset*module.y_offset;
            sum_omega += (module.x_offset * vy_i - module.y_offset * vx_i) / r2;
        }
        
        int n = modules_.size();
        double chassis_vx = sum_vx / n;
        double chassis_vy = sum_vy / n;
        double chassis_omega = sum_omega / n;

        double dt = period.toSec();
        double delta_x = (chassis_vx * cos(odom_theta_) - chassis_vy * sin(odom_theta_)) * dt;
        double delta_y = (chassis_vx * sin(odom_theta_) + chassis_vy * cos(odom_theta_)) * dt;
        double delta_th = chassis_omega * dt;

        odom_x_ += delta_x;
        odom_y_ += delta_y;
        odom_theta_ += delta_th;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom_theta_);
        geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);

        if (odom_pub_->trylock()) {
            odom_pub_->msg_.header.stamp = time;
            odom_pub_->msg_.header.frame_id = odom_frame_id_;
            odom_pub_->msg_.child_frame_id = base_frame_id_;
            
            odom_pub_->msg_.pose.pose.position.x = odom_x_;
            odom_pub_->msg_.pose.pose.position.y = odom_y_;
            odom_pub_->msg_.pose.pose.orientation = odom_quat;

            odom_pub_->msg_.twist.twist.linear.x = chassis_vx;
            odom_pub_->msg_.twist.twist.linear.y = chassis_vy;
            odom_pub_->msg_.twist.twist.angular.z = chassis_omega;

            odom_pub_->unlockAndPublish();
        }

        if (enable_odom_tf_) {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp = time;
            tf_msg.header.frame_id = odom_frame_id_;
            tf_msg.child_frame_id = base_frame_id_;
            tf_msg.transform.translation.x = odom_x_;
            tf_msg.transform.translation.y = odom_y_;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom_quat;
            
            tf_broadcaster_.sendTransform(tf_msg);
        }
    }

    void SentryChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        command_struct_ = *msg;
    }
    
    void SentryChassisController::starting(const ros::Time& time) {
        for(auto& m : modules_) {
            m.drive_pid.reset();
            m.pivot_pid.reset();
        }
    }
    
    void SentryChassisController::stopping(const ros::Time& time) {
        for(auto& m : modules_) {
            m.drive_joint.setCommand(0);
            m.pivot_joint.setCommand(0);
        }
    }

    // 新增：Dynamic Reconfigure 回调函数
    void SentryChassisController::reconfigCallback(sentry_chassis_controller::SentryPidsConfig& config, uint32_t level) {
        ROS_INFO_STREAM_THROTTLE(1.0, "Updating PIDs -> Drive P: " << config.drive_p << ", Pivot P: " << config.pivot_p);
        for (auto& module : modules_) {
            module.drive_pid.setGains(config.drive_p, config.drive_i, config.drive_d, 10.0, -10.0);
            module.pivot_pid.setGains(config.pivot_p, config.pivot_i, config.pivot_d, 10.0, -10.0);
        }
    }

} // <--- 关键！之前报错就是因为少了这里

// 插件导出宏必须在命名空间外面
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, controller_interface::ControllerBase)

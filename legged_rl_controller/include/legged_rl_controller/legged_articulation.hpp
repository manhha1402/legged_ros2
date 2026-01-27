#pragma once
#include "legged_rl_controller/isaaclab/assets/articulation/articulation.h"
#include "legged_ros2_controller/semantic_components/joint_interface.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace legged {

using TwistMsgSharedPtr = std::shared_ptr<geometry_msgs::msg::Twist>;
using CmdBuffer = realtime_tools::RealtimeBuffer<TwistMsgSharedPtr>;
using ImuMsgSharedPtr = std::shared_ptr<sensor_msgs::msg::Imu>;
using ImuBuffer = realtime_tools::RealtimeBuffer<ImuMsgSharedPtr>;

class LeggedArticulation : public isaaclab::Articulation {
public:
  LeggedArticulation(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                     bool use_sim,
                     std::shared_ptr<semantic_components::IMUSensor> imu_hardware,
                     std::shared_ptr<JointInterface> joint_interface,
                     std::shared_ptr<CmdBuffer> cmd_vel_buffer)
    : use_sim_(use_sim), imu_hardware_(imu_hardware), joint_interface_(joint_interface), cmd_vel_buffer_(cmd_vel_buffer) {

    if (use_sim_) {
      imu_buffer_ = std::make_shared<ImuBuffer>();
      imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_buffer_->writeFromNonRT(msg); });
      RCLCPP_INFO(node->get_logger(), "LeggedArticulation: Simulation mode - Direct IMU subscriber OK");
    } else {
      RCLCPP_INFO(node->get_logger(), "LeggedArticulation: Real Robot mode - Hardware IMU OK");
    }
  }

  void update() override {
    auto jp = joint_interface_->get_joint_position();
    auto jv = joint_interface_->get_joint_velocity();
    data.joint_pos = Eigen::Map<const Eigen::VectorXd>(jp.data(), jp.size()).cast<float>();
    data.joint_vel = Eigen::Map<const Eigen::VectorXd>(jv.data(), jv.size()).cast<float>();

    if (use_sim_) {
      ImuMsgSharedPtr imu_msg = *imu_buffer_->readFromRT();
      if (imu_msg) {
        data.root_ang_vel_b = Eigen::Vector3f(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        Eigen::Quaternionf q(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
        data.projected_gravity_b = q.conjugate() * data.GRAVITY_VEC_W;
      }
    } else if (imu_hardware_) {
      auto av = imu_hardware_->get_angular_velocity();
      data.root_ang_vel_b = Eigen::Vector3f(av[0], av[1], av[2]);
      auto quat = imu_hardware_->get_orientation();
      Eigen::Quaternionf q(quat[3], quat[0], quat[1], quat[2]);
      data.projected_gravity_b = q.conjugate() * data.GRAVITY_VEC_W;
    }

    TwistMsgSharedPtr cmd = *cmd_vel_buffer_->readFromRT();
    if(cmd) {
      data.command.lin_vel_x = cmd->linear.x; data.command.lin_vel_y = cmd->linear.y; data.command.ang_vel_z = cmd->angular.z;
    }
  }

private:
  bool use_sim_;
  std::shared_ptr<semantic_components::IMUSensor> imu_hardware_;
  std::shared_ptr<JointInterface> joint_interface_;
  std::shared_ptr<CmdBuffer> cmd_vel_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<ImuBuffer> imu_buffer_;
};

} // namespace legged
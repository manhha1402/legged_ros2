/**
 * @file legged_ros2_controller.cpp
 * @author xiaobaige (zitongbai@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-07-18
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "legged_ros2_controller/legged_ros2_controller.hpp"

#include <algorithm>
#include <memory>
#include <stddef.h>
#include <string>
#include <vector>

using config_type = controller_interface::interface_configuration_type;

namespace legged {

controller_interface::CallbackReturn LeggedController::on_init() {
  auto_declare<bool>("use_gains", false);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LeggedController::on_configure(const rclcpp_lifecycle::State &) {
  if (joint_names_.empty()) return CallbackReturn::ERROR;
  joint_interface_ = std::make_shared<JointInterface>(joint_names_);
  for (const auto &imu : imu_names_) {
    imu_interfaces_.emplace_back(std::make_shared<semantic_components::IMUSensor>(imu));
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration LeggedController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = config_type::INDIVIDUAL;
  bool use_gains = false; get_node()->get_parameter("use_gains", use_gains);

  if (use_gains) {
    conf.names = joint_interface_->get_command_interface_names();
  } else {
    for (const auto & j : joint_names_) {
      conf.names.push_back(j + "/position");
      conf.names.push_back(j + "/velocity");
      conf.names.push_back(j + "/effort");
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration LeggedController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration conf;
  conf.type = config_type::INDIVIDUAL;
  conf.names = joint_interface_->get_state_interface_names();

  bool use_gains = false; get_node()->get_parameter("use_gains", use_gains);
  if (use_gains) {
    for (const auto &imu_if : imu_interfaces_) {
      auto imu_if_names = imu_if->get_state_interface_names();
      conf.names.insert(conf.names.end(), imu_if_names.begin(), imu_if_names.end());
    }
  }
  return conf;
}

controller_interface::CallbackReturn LeggedController::on_activate(const rclcpp_lifecycle::State &) {
  bool success = true;
  success &= joint_interface_->assign_loaned_state_interfaces(state_interfaces_);

  bool use_gains = false; get_node()->get_parameter("use_gains", use_gains);
  if (use_gains) {
    for (auto &imu_if : imu_interfaces_) success &= imu_if->assign_loaned_state_interfaces(state_interfaces_);
  }
  success &= joint_interface_->assign_loaned_command_interfaces(command_interfaces_);
  return success ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

controller_interface::CallbackReturn LeggedController::on_deactivate(const rclcpp_lifecycle::State &) {
  joint_interface_->release_interfaces();
  bool use_gains = false; get_node()->get_parameter("use_gains", use_gains);
  if (use_gains) {
    for (auto &imu_if : imu_interfaces_) imu_if->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
LeggedController::update(const rclcpp::Time & /*time*/,
                         const rclcpp::Duration & /*period*/) {

  return controller_interface::return_type::OK;
}

} // namespace legged

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(legged::LeggedController,
                       controller_interface::ControllerInterface)

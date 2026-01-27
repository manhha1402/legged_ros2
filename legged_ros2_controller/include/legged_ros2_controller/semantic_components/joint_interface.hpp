#pragma once
#include <limits>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include "rclcpp/macros.hpp"
#include "controller_interface/helpers.hpp"
#include "controller_interface/controller_interface.hpp"
#include "legged_ros2_control/legged_system_interface.hpp"

namespace legged {

struct JointCommandHandles {
  hardware_interface::LoanedCommandInterface* pos = nullptr;
  hardware_interface::LoanedCommandInterface* vel = nullptr;
  hardware_interface::LoanedCommandInterface* effort = nullptr;
  hardware_interface::LoanedCommandInterface* kp = nullptr;
  hardware_interface::LoanedCommandInterface* kd = nullptr;
};

class JointInterface {
public:
  explicit JointInterface(const std::vector<std::string> & joint_names) : joint_names_(joint_names) {
    joint_num_ = joint_names_.size();
    state_interface_names_.reserve(joint_num_ * 3);
    command_interface_names_.reserve(joint_num_ * 5);

    for (const auto & joint_name : joint_names_) {
      state_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      state_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);

      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_KP);
      command_interface_names_.push_back(joint_name + "/" + hardware_interface::HW_IF_KD);
    }
    joint_handles_.resize(joint_num_);
  }

  bool assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface> & state_interfaces) {
    return controller_interface::get_ordered_interfaces<hardware_interface::LoanedStateInterface>(
      state_interfaces, state_interface_names_, "", state_interfaces_);
  }

  bool assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces) {
    for (size_t i = 0; i < joint_num_; ++i) {
      joint_handles_[i] = JointCommandHandles();
      for (auto & interface : command_interfaces) {
        const std::string& name = interface.get_name();
        if (name == joint_names_[i] + "/" + hardware_interface::HW_IF_POSITION) { joint_handles_[i].pos = &interface; }
        else if (name == joint_names_[i] + "/" + hardware_interface::HW_IF_VELOCITY) { joint_handles_[i].vel = &interface; }
        else if (name == joint_names_[i] + "/" + hardware_interface::HW_IF_EFFORT) { joint_handles_[i].effort = &interface; }
        else if (name == joint_names_[i] + "/" + hardware_interface::HW_IF_KP) { joint_handles_[i].kp = &interface; }
        else if (name == joint_names_[i] + "/" + hardware_interface::HW_IF_KD) { joint_handles_[i].kd = &interface; }
      }
    }
    return true;
  }

  void release_interfaces() {
    state_interfaces_.clear();
    std::fill(joint_handles_.begin(), joint_handles_.end(), JointCommandHandles());
  }

  void set_joint_command(const std::vector<float> & pos, const std::vector<float> & vel,
                         const std::vector<float> & ff, const std::vector<float> & kp,
                         const std::vector<float> & kd) {
    for(size_t i=0; i<joint_num_; i++){
      if (joint_handles_[i].pos) joint_handles_[i].pos->set_value(static_cast<double>(pos[i]));
      if (joint_handles_[i].vel) joint_handles_[i].vel->set_value(static_cast<double>(vel[i]));
      if (joint_handles_[i].effort) joint_handles_[i].effort->set_value(static_cast<double>(ff[i]));
      if (joint_handles_[i].kp) joint_handles_[i].kp->set_value(static_cast<double>(kp[i]));
      if (joint_handles_[i].kd) joint_handles_[i].kd->set_value(static_cast<double>(kd[i]));
    }
  }

  void set_joint_command(const std::vector<double> & pos, const std::vector<double> & vel,
                         const std::vector<double> & ff, const std::vector<double> & kp,
                         const std::vector<double> & kd) {
    for(size_t i=0; i<joint_num_; i++){
      if (joint_handles_[i].pos) joint_handles_[i].pos->set_value(pos[i]);
      if (joint_handles_[i].vel) joint_handles_[i].vel->set_value(vel[i]);
      if (joint_handles_[i].effort) joint_handles_[i].effort->set_value(ff[i]);
      if (joint_handles_[i].kp) joint_handles_[i].kp->set_value(kp[i]);
      if (joint_handles_[i].kd) joint_handles_[i].kd->set_value(kd[i]);
    }
  }

  std::vector<double> get_joint_position() const {
    std::vector<double> positions(joint_num_);
    for(size_t i=0; i<joint_num_; i++) positions[i] = state_interfaces_[3*i].get().get_value();
    return positions;
  }

  std::vector<double> get_joint_velocity() const {
    std::vector<double> velocities(joint_num_);
    for(size_t i=0; i<joint_num_; i++) velocities[i] = state_interfaces_[3*i+1].get().get_value();
    return velocities;
  }

  std::vector<std::string> get_state_interface_names() { return state_interface_names_; }
  std::vector<std::string> get_command_interface_names() { return command_interface_names_; }

private:
  std::vector<std::string> joint_names_;
  size_t joint_num_;
  std::vector<std::string> state_interface_names_;
  std::vector<std::string> command_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
  std::vector<JointCommandHandles> joint_handles_;
};

} // namespace legged
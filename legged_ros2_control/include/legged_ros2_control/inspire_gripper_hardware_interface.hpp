#pragma once
#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace vm_ros2_control
{
class InspireGripperHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(InspireGripperHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string port_;
  int baudrate_;
  int serial_fd_{-1};

  double hw_command_{0.0};
  double hw_position_{0.0};
  double last_command_{-1.0};
  double hw_velocity_{0.0};
  double hw_reactivate_cmd_{0.0};
  double hw_reactivate_res_{0.0};
  bool is_hardware_connected_{false};

  bool open_serial();
  void close_serial();
  void send_inspire_command(uint8_t cmd_hex, const std::vector<uint8_t> & data);
  bool read_serial_response(uint8_t expected_cmd);
};
}  // namespace vm_ros2_control

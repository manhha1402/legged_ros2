#include "legged_ros2_control/inspire_gripper_hardware_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <numeric>

namespace vm_ros2_control
{

hardware_interface::CallbackReturn InspireGripperHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read infor from file Xacro/URDF
  port_ = info_.hardware_parameters.at("serial_port");
  baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));

  // Check config
  if (info_.joints.size() != 1) {
    RCLCPP_FATAL(rclcpp::get_logger("InspireGripperHardwareInterface"), "HWI gripper only support 1 joint!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> InspireGripperHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_position_));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> InspireGripperHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.joints[0].name, hardware_interface::HW_IF_POSITION, &hw_command_));

  if (info_.gpios.size() > 0) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.gpios[0].name, "reactivate_gripper_cmd", &hw_reactivate_cmd_));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.gpios[0].name, "reactivate_gripper_response", &hw_reactivate_res_));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn InspireGripperHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("InspireGripperHardwareInterface"), "Attempting to connect to gripper on %s", port_.c_str());

  if (open_serial()) {
    is_hardware_connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("InspireGripperHardwareInterface"), "SUCCESS: Connected to real gripper.");
  } else {
    is_hardware_connected_ = false;
    RCLCPP_WARN(rclcpp::get_logger("InspireGripperHardwareInterface"),
      "FAILED: Could not open %s. Running in VIRTUAL MODE for this gripper.", port_.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireGripperHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type InspireGripperHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  hw_position_ = hw_command_; 
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InspireGripperHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (is_hardware_connected_) {
    if (std::abs(hw_command_ - last_command_) < 0.1) {
      return hardware_interface::return_type::OK;
    }

    uint8_t cmd_hex;
    std::vector<uint8_t> data;
    uint16_t speed = 500; // 50%
    uint16_t force = 500; // ~effort = 50

    if (hw_command_ > 0.4) { // Open Command
      cmd_hex = 0x11;
      data.push_back(speed & 0xFF);
      data.push_back((speed >> 8) & 0xFF);
    } else { // Close Command
      cmd_hex = 0x10;
      data.push_back(speed & 0xFF);
      data.push_back((speed >> 8) & 0xFF);
      data.push_back(force & 0xFF);
      data.push_back((force >> 8) & 0xFF);
    }

    send_inspire_command(cmd_hex, data);
  }
  last_command_ = hw_command_;

  return hardware_interface::return_type::OK;
}

// ======================== SERIAL LOGIC ========================

bool InspireGripperHardwareInterface::read_serial_response(uint8_t expected_cmd)
{
  if (serial_fd_ < 0) return false;

  uint8_t buffer[32];
  usleep(10000);

  int bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));

  if (bytes_read <= 0) {
    return false;
  }

  for (int i = 0; i < bytes_read - 4; ++i) {
    if (buffer[i] == 0xEB && buffer[i+1] == 0x90) {
        uint8_t received_cmd = buffer[i+4];
        if (received_cmd == expected_cmd) {
            return true;
        }
    }
  }

  return false;
}

bool InspireGripperHardwareInterface::open_serial()
{
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("InspireGripperHardwareInterface"), "Serial connect error!");
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) return false;

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;

  tcsetattr(serial_fd_, TCSANOW, &tty);
  return true;
}

void InspireGripperHardwareInterface::close_serial()
{
  if (serial_fd_ >= 0) close(serial_fd_);
}

void InspireGripperHardwareInterface::send_inspire_command(uint8_t cmd_hex, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> frame = {0xEB, 0x90, 0x01, (uint8_t)(data.size() + 1), cmd_hex};
  frame.insert(frame.end(), data.begin(), data.end());

  uint8_t checksum = 0;
  for (size_t i = 2; i < frame.size(); ++i) checksum += frame[i];
  frame.push_back(checksum);

  ::write(serial_fd_, frame.data(), frame.size());
}

} // namespace vm_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(vm_ros2_control::InspireGripperHardwareInterface, hardware_interface::SystemInterface)
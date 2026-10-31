// Copyright 2026 <Robomanipal>

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "motion_sdk/servo.hpp"
#include "motion_sdk/usb_serial.hpp"

namespace arm_servo_controller
{

class ArmServoHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  size_t num_joints_;

  std::vector<double> position_state_;
  std::vector<double> velocity_state_;
  std::vector<double> effort_state_;
  std::vector<double> position_command_;

  // Per-joint conversion between the ros2_control unit and servo degrees:
  //   servo_deg  = command * deg_per_unit_[i] + deg_offset_[i]
  //   state      = (servo_deg - deg_offset_[i]) / deg_per_unit_[i]
  // Revolute joints (rad):      deg_per_unit = 180/pi, offset = 0   (defaults)
  // Prismatic joints (metres):  set <param name="deg_per_meter"> in the xacro
  std::vector<double> deg_per_unit_;
  std::vector<double> deg_offset_;

  std::string port_;
  std::vector<int> servo_ids_;

  std::unique_ptr<motion_sdk::USBSerial> serial_;
  std::vector<std::unique_ptr<motion_sdk::Servo>> servos_;

  rclcpp::Logger logger_{rclcpp::get_logger("arm_servo_hardware")};
};

}  // namespace arm_servo_controller
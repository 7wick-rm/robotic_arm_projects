// Copyright 2026 <Robomanipal>
#include "arm_servo_controller/arm_servo_hardware.hpp"
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace arm_servo_controller
{

hardware_interface::CallbackReturn ArmServoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  num_joints_ = info_.joints.size();

  position_state_.resize(num_joints_, 0.0);
  velocity_state_.resize(num_joints_, 0.0);
  effort_state_.resize(num_joints_, 0.0);
  position_command_.resize(num_joints_, 0.0);

  port_ = info_.hardware_parameters.at("port");

  serial_ = std::make_unique<motion_sdk::USBSerial>(port_, 115200);
  if (!serial_->open())
  {
    RCLCPP_ERROR(logger_, "Serial open failed");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < num_joints_; ++i)
  {
    const auto & params = info_.joints[i].parameters;

    int id = std::stoi(params.at("id"));
    servo_ids_.push_back(id);

    // Unit conversion: revolute (rad) by default, prismatic if deg_per_meter is given
    double deg_per_unit = 180.0 / M_PI;
    if (params.count("deg_per_meter"))
    {
      deg_per_unit = std::stod(params.at("deg_per_meter"));
      if (deg_per_unit == 0.0)
      {
        RCLCPP_ERROR(logger_, "deg_per_meter must be non-zero (joint %s)",
                     info_.joints[i].name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    deg_per_unit_.push_back(deg_per_unit);
    deg_offset_.push_back(params.count("deg_offset") ? std::stod(params.at("deg_offset")) : 0.0);

    auto servo = std::make_unique<motion_sdk::Servo>(*serial_, id);
    if (!servo->ping())
    {
      RCLCPP_ERROR(logger_, "Servo ping failed id=%d", id);
      return CallbackReturn::ERROR;
    }

    // Start with command == current position. Otherwise write() would send 0
    // to every servo from the moment the controller manager starts.
    float deg{};
    if (servo->getPosition(deg))
    {
      position_state_[i] = (deg - deg_offset_[i]) / deg_per_unit_[i];
      position_command_[i] = position_state_[i];
    }
    else
    {
      RCLCPP_WARN(logger_, "Could not read initial position of servo id=%d", id);
    }

    servos_.push_back(std::move(servo));
  }

  RCLCPP_INFO(logger_, "Arm servo hardware initialized (%ld joints)", num_joints_);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmServoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    si.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_state_[i]);
    si.emplace_back(info_.joints[i].name, HW_IF_VELOCITY, &velocity_state_[i]);
    si.emplace_back(info_.joints[i].name, HW_IF_EFFORT, &effort_state_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> ArmServoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    ci.emplace_back(info_.joints[i].name, HW_IF_POSITION, &position_command_[i]);
  }
  return ci;
}

hardware_interface::return_type ArmServoHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Position only: every extra blocking serial round-trip eats into the
  // controller_manager cycle and caused visible jitter under trajectory tracking.
  for (size_t i = 0; i < num_joints_; ++i)
  {
    float deg{};
    if (servos_[i]->getPosition(deg))
    {
      position_state_[i] = (deg - deg_offset_[i]) / deg_per_unit_[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmServoHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    servos_[i]->setPosition(
      static_cast<float>(position_command_[i] * deg_per_unit_[i] + deg_offset_[i]));
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_servo_controller

PLUGINLIB_EXPORT_CLASS(arm_servo_controller::ArmServoHardware, hardware_interface::SystemInterface)
// Copyright 2026 <Robomanipal>
#include "arm_servo_controller/arm_servo_hardware.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <pluginlib/class_list_macros.hpp>
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace arm_servo_controller
{

// Servo angle -> Jaw position (m). Clamped, so the reported state can never leave
// the open..closed range even if the servo overshoots a degree or two.
double ArmServoHardware::gripperPosFromDeg(const GripperCfg & g, double deg) const
{
  double t = (deg - g.open_deg) / (g.close_deg - g.open_deg);  // 0 = open, 1 = closed
  t = std::clamp(t, 0.0, 1.0);
  return g.open_pos + t * (g.close_pos - g.open_pos);
}

double ArmServoHardware::gripperFraction(const GripperCfg & g, double pos) const
{
  return (pos - g.open_pos) / (g.close_pos - g.open_pos);
}

// STM command: "GRIPPER <id> 0" = open, "GRIPPER <id> 1" = close.
// The bus is strict request/response, so the reply line MUST be consumed here,
// otherwise the next GET_POS would read it as its own answer.
bool ArmServoHardware::sendGripper(size_t i, bool close)
{
  std::ostringstream cmd;
  cmd << "GRIPPER " << servo_ids_[i] << " " << (close ? 1 : 0);

  if (!serial_->writeLine(cmd.str()))
  {
    return false;
  }

  std::string resp;
  if (!serial_->readLine(resp))
  {
    return false;
  }

  if (resp != "OK")
  {
    RCLCPP_WARN(logger_, "Unexpected reply to '%s': '%s'", cmd.str().c_str(), resp.c_str());
  }
  RCLCPP_INFO(logger_, "Sent '%s'", cmd.str().c_str());
  return true;
}

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
    auto get = [&](const char * key, double fallback)
    {
      return params.count(key) ? std::stod(params.at(key)) : fallback;
    };

    int id = std::stoi(params.at("id"));
    servo_ids_.push_back(id);

    const bool is_gripper = params.count("type") && params.at("type") == "binary_gripper";
    is_gripper_.push_back(is_gripper ? 1 : 0);

    GripperCfg g;
    if (is_gripper)
    {
      g.open_deg = get("open_deg", g.open_deg);
      g.close_deg = get("close_deg", g.close_deg);
      g.open_pos = get("open_pos", g.open_pos);
      g.close_pos = get("close_pos", g.close_pos);
      g.hysteresis = std::clamp(get("gripper_hysteresis", g.hysteresis), 0.0, 0.45);
      if (g.open_deg == g.close_deg || g.open_pos == g.close_pos)
      {
        RCLCPP_ERROR(logger_, "Gripper joint %s: open and close values must differ",
                     info_.joints[i].name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    gripper_.push_back(g);

    // Unit conversion for ordinary joints: revolute (rad) unless deg_per_meter is given
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
    deg_offset_.push_back(get("deg_offset", 0.0));

    auto servo = std::make_unique<motion_sdk::Servo>(*serial_, id);

    // Liveness check with a read-only query. Servo::ping() sends "SET_POS <id> " with no
    // angle; the STM can read that as a move command (to 0 deg), which drives the gripper
    // past its physical end stop at launch. GET_POS cannot move anything.
    float deg{};
    if (!servo->getPosition(deg))
    {
      RCLCPP_ERROR(logger_, "Servo id=%d did not answer GET_POS", id);
      return CallbackReturn::ERROR;
    }

    // Start with command == current position. Otherwise write() would send 0
    // to every servo from the moment the controller manager starts.
    if (is_gripper)
    {
      position_state_[i] = gripperPosFromDeg(gripper_[i], deg);
      // Remember which side the gripper is on, so nothing is re-sent at startup.
      const double f = gripperFraction(gripper_[i], position_state_[i]);
      gripper_[i].last_sent = f < 0.5 - g.hysteresis ? 0 : (f > 0.5 + g.hysteresis ? 1 : -1);
      RCLCPP_INFO(logger_, "Gripper %s: servo at %.1f deg -> Jaw %.4f m (%s)",
                  info_.joints[i].name.c_str(), static_cast<double>(deg), position_state_[i],
                  gripper_[i].last_sent == 0 ? "open" :
                  (gripper_[i].last_sent == 1 ? "closed" : "between"));
    }
    else
    {
      position_state_[i] = (deg - deg_offset_[i]) / deg_per_unit_[i];
    }
    position_command_[i] = position_state_[i];

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
    if (!servos_[i]->getPosition(deg))
    {
      continue;
    }
    position_state_[i] = is_gripper_[i]
      ? gripperPosFromDeg(gripper_[i], deg)
      : (deg - deg_offset_[i]) / deg_per_unit_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmServoHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (is_gripper_[i])
    {
      // Binary gripper: turn the commanded position into open / close, and send a
      // command only when the wanted state CHANGES. Re-sending every cycle would keep
      // restarting the STM's load-limited close.
      auto & g = gripper_[i];
      if (g.cooldown > 0)
      {
        --g.cooldown;
        continue;
      }

      const double f = gripperFraction(g, position_command_[i]);  // 0 open ... 1 closed
      int want = -1;
      if (f < 0.5 - g.hysteresis)
      {
        want = 0;
      }
      else if (f > 0.5 + g.hysteresis)
      {
        want = 1;
      }

      if (want != -1 && want != g.last_sent)
      {
        if (sendGripper(i, want == 1))
        {
          g.last_sent = want;
        }
        else
        {
          RCLCPP_WARN(logger_, "GRIPPER command failed (id=%d), retrying shortly", servo_ids_[i]);
          g.cooldown = 25;  // ~0.5 s at 50 Hz
        }
      }
      continue;
    }

    servos_[i]->setPosition(
      static_cast<float>(position_command_[i] * deg_per_unit_[i] + deg_offset_[i]));
  }
  return hardware_interface::return_type::OK;
}

}  // namespace arm_servo_controller

PLUGINLIB_EXPORT_CLASS(arm_servo_controller::ArmServoHardware, hardware_interface::SystemInterface)
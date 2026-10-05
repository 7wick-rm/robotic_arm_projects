// Copyright 2026 <Robomanipal>

#pragma once

#include <sstream>
#include <string>

#include "motion_sdk/usb_serial.hpp"

namespace motion_sdk
{

// MG995 is a non-feedback, PWM-driven servo switched by the STM firmware.
// Unlike Servo (AX18A), it cannot report position/effort/speed and only
// accepts a binary open/closed state -- no SET_POS/GET_POS protocol.
class GripperServo
{
public:
  GripperServo(USBSerial & serial, int id) : serial_(serial), id_(id) {}

  // state: true = open, false = closed
  bool setState(bool open)
  {
    std::ostringstream cmd;
    cmd << "SET_GRIP " << id_ << " " << (open ? 1 : 0);

    if (!serial_.writeLine(cmd.str()))
    {
      return false;
    }

    std::string resp;
    if (!serial_.readLine(resp))
    {
      return false;
    }

    return resp == "OK";
  }

private:
  USBSerial & serial_;
  int id_;
};

}  // namespace motion_sdk
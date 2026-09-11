// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include "robosoft_simulator/can_protocol.hpp"

#include <ros2_isobus/msg/isobus_frame.hpp>

namespace robosoft_simulator
{

inline ros2_isobus::msg::IsobusFrame toIsobusFrame(
  const can_protocol::Frame & frame)
{
  ros2_isobus::msg::IsobusFrame message;
  message.priority = frame.priority;
  message.page = false;
  message.pgn = frame.pgn();
  message.sa = frame.sa;
  message.pf = frame.pf;
  message.ps = frame.ps;
  message.dlc = frame.dlc;
  message.data = frame.data;
  return message;
}

inline can_protocol::Frame fromIsobusFrame(
  const ros2_isobus::msg::IsobusFrame & message)
{
  can_protocol::Frame frame;
  frame.priority = message.priority;
  frame.pf = message.pf != 0U ? message.pf :
    static_cast<std::uint8_t>((message.pgn >> 8U) & 0xFFU);
  frame.ps = message.ps != 0U ? message.ps :
    static_cast<std::uint8_t>(message.pgn & 0xFFU);
  frame.sa = message.sa;
  frame.dlc = message.dlc == 0U ? 8U : std::min<std::uint8_t>(message.dlc, 8U);
  frame.data = message.data;
  return frame;
}

}  // namespace robosoft_simulator

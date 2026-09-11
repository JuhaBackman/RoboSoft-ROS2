// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include "robosoft_simulator/can_protocol.hpp"

#include <cstdint>

namespace robosoft_simulator::aki_can_devices
{

using can_protocol::Frame;
using can_protocol::Payload;

constexpr std::uint8_t kGuidanceSa = 0x13;
constexpr std::uint8_t kRemoteSa = 0x80;
constexpr std::uint8_t kUvcSa = 0x81;
constexpr Payload kGuidanceName{0xA0, 0x00, 0x19, 0x00, 0xAA, 0xA0, 0x00, 0x06};
constexpr Payload kRemoteName{0x01, 0x00, 0x60, 0x07, 0x00, 0x90, 0x01, 0xA0};
constexpr Payload kUvcName{0x02, 0x00, 0x60, 0x07, 0x00, 0x90, 0x01, 0xA0};

inline Frame pdu2(std::uint32_t pgn, std::uint8_t sa, const Payload & data)
{
  Frame frame;
  frame.priority = 3;
  frame.pf = static_cast<std::uint8_t>((pgn >> 8U) & 0xFFU);
  frame.ps = static_cast<std::uint8_t>(pgn & 0xFFU);
  frame.sa = sa;
  frame.data = data;
  return frame;
}

inline Frame pdu1(
  std::uint8_t pf, std::uint8_t destination, std::uint8_t sa,
  const Payload & data)
{
  Frame frame;
  frame.priority = 3;
  frame.pf = pf;
  frame.ps = destination;
  frame.sa = sa;
  frame.data = data;
  return frame;
}

inline bool requestsAddressClaim(const Frame & frame, std::uint8_t device_sa)
{
  if (frame.pf != 0xEA || (frame.ps != 0xFF && frame.ps != device_sa)) {
    return false;
  }
  const auto requested_pgn = static_cast<std::uint32_t>(frame.data[0]) |
    static_cast<std::uint32_t>(frame.data[1]) << 8U |
    static_cast<std::uint32_t>(frame.data[2]) << 16U;
  return requested_pgn == 0xEE00U;
}

}  // namespace robosoft_simulator::aki_can_devices

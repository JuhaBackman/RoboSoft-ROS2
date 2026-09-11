// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "robosoft_simulator/can_protocol.hpp"
#include "robosoft_simulator/isobus_frame.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace protocol = robosoft_simulator::can_protocol;

namespace
{
std::uint32_t getBits(
  const protocol::Payload & data, unsigned start, unsigned length)
{
  std::uint32_t result = 0;
  for (unsigned bit = 0; bit < length; ++bit) {
    if ((data[(start + bit) / 8] & (1U << ((start + bit) % 8))) != 0U) {
      result |= 1U << bit;
    }
  }
  return result;
}
}  // namespace

TEST(AkiCanProtocol, PreservesExtendedCanIdentifierFields)
{
  protocol::Frame original;
  original.priority = 3;
  original.pf = 0xAC;
  original.ps = 0x1C;
  original.sa = 0x13;
  const auto decoded = protocol::parseCanId(protocol::canId(original));
  EXPECT_EQ(decoded.priority, original.priority);
  EXPECT_EQ(decoded.pf, original.pf);
  EXPECT_EQ(decoded.ps, original.ps);
  EXPECT_EQ(decoded.sa, original.sa);
  EXPECT_EQ(decoded.pgn(), 0xAC00U);
}

TEST(AkiCanProtocol, ConvertsFramesForRos2IsobusCanBridge)
{
  protocol::Frame original;
  original.priority = 3;
  original.pf = 0xFF;
  original.ps = 0x31;
  original.sa = 0x81;
  original.data[0] = 0x07;

  const auto message = robosoft_simulator::toIsobusFrame(original);
  EXPECT_EQ(message.pgn, 0xFF31U);
  EXPECT_EQ(message.pf, 0xFFU);
  EXPECT_EQ(message.ps, 0x31U);
  EXPECT_EQ(message.sa, 0x81U);
  const auto decoded = robosoft_simulator::fromIsobusFrame(message);
  EXPECT_EQ(protocol::canId(decoded), protocol::canId(original));
  EXPECT_EQ(decoded.data, original.data);
}

TEST(AkiCanProtocol, EncodesAkiRemoteControls)
{
  const auto data = protocol::encodeAkiRemoteControls(3, true, true, true);
  EXPECT_EQ(getBits(data, 0, 2), 1U);
  EXPECT_EQ(getBits(data, 2, 2), 1U);
  EXPECT_EQ(getBits(data, 14, 2), 1U);
  EXPECT_EQ(getBits(data, 22, 2), 1U);
}

TEST(AkiCanProtocol, DecodesAkiRemoteInputs)
{
  const auto encoded_axes = protocol::encodeRemoteAxes(-321, 654, 987, -123);
  const auto axes = protocol::decodeRemoteAxes(encoded_axes);
  EXPECT_EQ(axes.joystick_1_x, -321);
  EXPECT_EQ(axes.joystick_1_y, 654);
  EXPECT_EQ(axes.joystick_2_x, 987);
  EXPECT_EQ(axes.joystick_2_y, -123);

  const auto encoded_controls =
    protocol::encodeAkiRemoteControls(3, true, false, true);
  const auto controls = protocol::decodeAkiRemoteControls(encoded_controls);
  EXPECT_EQ(controls.mode, 3U);
  EXPECT_TRUE(controls.safety);
  EXPECT_FALSE(controls.start);
  EXPECT_TRUE(controls.deadman);
}

TEST(AkiCanProtocol, DecodesClassThreeCommands)
{
  protocol::Payload speed{};
  speed[2] = 0xDC;
  speed[3] = 0x05;
  EXPECT_NEAR(protocol::decodeCruiseCommand(speed), 1.5, 1e-9);

  protocol::Payload curvature{};
  const auto raw = static_cast<std::uint16_t>((8032.0 - 2.0) * 4.0);
  curvature[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  curvature[1] = static_cast<std::uint8_t>(raw >> 8U);
  EXPECT_NEAR(protocol::decodeCurvatureCommand(curvature), 2.0, 1e-9);
}

TEST(AkiCanProtocol, EncodesWheelSpeedForRos2IsobusDecoder)
{
  const auto data = protocol::encodeWheelSpeed(-1.25, -12.5, true);
  const auto speed_raw = static_cast<std::uint16_t>(data[0]) |
    static_cast<std::uint16_t>(data[1]) << 8U;
  const auto distance_raw = static_cast<std::uint32_t>(data[2]) |
    static_cast<std::uint32_t>(data[3]) << 8U |
    static_cast<std::uint32_t>(data[4]) << 16U |
    static_cast<std::uint32_t>(data[5]) << 24U;
  EXPECT_EQ(speed_raw, 1250U);
  EXPECT_EQ(static_cast<std::int32_t>(distance_raw), -12500);
  EXPECT_EQ(data[7] & 0x03U, 0U);
  EXPECT_EQ((data[7] >> 2U) & 0x03U, 1U);
}

TEST(AkiCanProtocol, CompletesUvcFeedbackWithInactiveState)
{
  const auto active = protocol::encodeUvcStatus(true, 2.5);
  EXPECT_EQ(active[0], 0x07U);
  EXPECT_EQ(active[5], 25U);
  const auto complete = protocol::encodeUvcStatus(false, 20.0);
  EXPECT_EQ(complete[0], 0U);
  EXPECT_EQ(complete[5], 200U);
}

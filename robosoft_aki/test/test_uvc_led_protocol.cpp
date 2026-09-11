// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "uvc_led_protocol.hpp"

TEST(UvcLedProtocol, EncodesCommandPayload)
{
  robosoft_aki::uvc_led_protocol::Command command;
  const auto data = robosoft_aki::uvc_led_protocol::encodeCommand(command);
  EXPECT_EQ(data[0], 40);
  EXPECT_EQ(data[1], 30);
  EXPECT_EQ(data[2], 0xF4);
  EXPECT_EQ(data[3], 0x01);
  EXPECT_EQ(data[4], 200);
  EXPECT_EQ(data[5], 150);
  EXPECT_EQ(data[6], 0);
}

TEST(UvcLedProtocol, DecodesStatusPayload)
{
  const std::array<uint8_t, 8> data{0x05, 0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0};
  const auto status = robosoft_aki::uvc_led_protocol::decodeStatus(data);
  EXPECT_TRUE(status.camera_light_on);
  EXPECT_FALSE(status.camera_trigger_on);
  EXPECT_TRUE(status.uvc_light_on);
  EXPECT_TRUE(status.active());
  EXPECT_EQ(status.uvc_distance_front_mm, 0x1234);
  EXPECT_EQ(status.uvc_distance_back_mm, 0x5678);
  EXPECT_EQ(status.time_on_ds, 0x9ABC);
}

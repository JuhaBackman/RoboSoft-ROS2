// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <array>
#include <cstdint>

namespace robosoft_aki::uvc_led_protocol
{

constexpr uint32_t kCommandPgn = 0xFF30;
constexpr uint32_t kStatusPgn = 0xFF31;

struct Command
{
  uint8_t camera_light_time_ds{40};
  uint8_t camera_trigger_time_ds{30};
  uint16_t camera_light_power{500};
  uint8_t uvc_light_time_ds{200};
  uint16_t uvc_light_distance_mm{150};
};

struct Status
{
  bool camera_light_on{false};
  bool camera_trigger_on{false};
  bool uvc_light_on{false};
  uint16_t uvc_distance_front_mm{0};
  uint16_t uvc_distance_back_mm{0};
  uint16_t time_on_ds{0};

  bool active() const
  {
    return camera_light_on || camera_trigger_on || uvc_light_on;
  }
};

inline std::array<uint8_t, 8> encodeCommand(const Command & command)
{
  return {
    command.camera_light_time_ds,
    command.camera_trigger_time_ds,
    static_cast<uint8_t>(command.camera_light_power & 0xFFU),
    static_cast<uint8_t>(command.camera_light_power >> 8U),
    command.uvc_light_time_ds,
    static_cast<uint8_t>(command.uvc_light_distance_mm & 0xFFU),
    static_cast<uint8_t>(command.uvc_light_distance_mm >> 8U),
    0U};
}

inline Status decodeStatus(const std::array<uint8_t, 8> & data)
{
  Status status;
  status.camera_light_on = (data[0] & 0x01U) != 0U;
  status.camera_trigger_on = (data[0] & 0x02U) != 0U;
  status.uvc_light_on = (data[0] & 0x04U) != 0U;
  status.uvc_distance_front_mm =
    static_cast<uint16_t>(data[1]) |
    static_cast<uint16_t>(data[2]) << 8U;
  status.uvc_distance_back_mm =
    static_cast<uint16_t>(data[3]) |
    static_cast<uint16_t>(data[4]) << 8U;
  status.time_on_ds =
    static_cast<uint16_t>(data[5]) |
    static_cast<uint16_t>(data[6]) << 8U;
  return status;
}

}  // namespace robosoft_aki::uvc_led_protocol

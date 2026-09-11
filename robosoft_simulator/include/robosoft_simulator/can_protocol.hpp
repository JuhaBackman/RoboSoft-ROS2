// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace robosoft_simulator::can_protocol
{

using Payload = std::array<std::uint8_t, 8>;

struct Frame
{
  std::uint8_t priority{6};
  std::uint8_t pf{0};
  std::uint8_t ps{0};
  std::uint8_t sa{0};
  std::uint8_t dlc{8};
  Payload data{};

  std::uint32_t pgn() const
  {
    return pf < 240U ? static_cast<std::uint32_t>(pf) << 8U :
           (static_cast<std::uint32_t>(pf) << 8U) | ps;
  }
};

inline std::uint32_t canId(const Frame & frame)
{
  return (static_cast<std::uint32_t>(frame.priority & 0x07U) << 26U) |
         (static_cast<std::uint32_t>(frame.pf) << 16U) |
         (static_cast<std::uint32_t>(frame.ps) << 8U) | frame.sa;
}

inline Frame parseCanId(std::uint32_t id)
{
  Frame frame;
  frame.priority = static_cast<std::uint8_t>((id >> 26U) & 0x07U);
  frame.pf = static_cast<std::uint8_t>((id >> 16U) & 0xFFU);
  frame.ps = static_cast<std::uint8_t>((id >> 8U) & 0xFFU);
  frame.sa = static_cast<std::uint8_t>(id & 0xFFU);
  return frame;
}

inline void setBits(Payload & data, unsigned start, unsigned length, std::uint32_t value)
{
  for (unsigned bit = 0; bit < length && start + bit < 64U; ++bit) {
    const auto mask = static_cast<std::uint8_t>(1U << ((start + bit) % 8U));
    auto & byte = data[(start + bit) / 8U];
    if ((value & (1U << bit)) != 0U) {
      byte |= mask;
    } else {
      byte &= static_cast<std::uint8_t>(~mask);
    }
  }
}

inline std::uint32_t getBits(
  const Payload & data, unsigned start, unsigned length)
{
  std::uint32_t value = 0U;
  for (unsigned bit = 0; bit < length && start + bit < 64U; ++bit) {
    if ((data[(start + bit) / 8U] & (1U << ((start + bit) % 8U))) != 0U) {
      value |= 1U << bit;
    }
  }
  return value;
}

inline std::int16_t decodeSignedAxis(
  const Payload & data, unsigned value_bit, unsigned sign_bit)
{
  const auto magnitude = static_cast<std::int16_t>(getBits(data, value_bit, 10));
  return getBits(data, sign_bit, 2) == 1U ? -magnitude : magnitude;
}

struct RemoteAxes
{
  std::int16_t joystick_1_x{0};
  std::int16_t joystick_1_y{0};
  std::int16_t joystick_2_x{0};
  std::int16_t joystick_2_y{0};
};

inline RemoteAxes decodeRemoteAxes(const Payload & data)
{
  return {
    decodeSignedAxis(data, 6, 4),
    decodeSignedAxis(data, 22, 20),
    decodeSignedAxis(data, 54, 52),
    decodeSignedAxis(data, 38, 36)};
}

struct AkiRemoteControls
{
  std::uint8_t mode{0};
  bool safety{false};
  bool start{false};
  bool deadman{false};
};

inline AkiRemoteControls decodeAkiRemoteControls(const Payload & data)
{
  AkiRemoteControls controls;
  controls.safety = getBits(data, 0, 2) == 1U;
  controls.start = getBits(data, 2, 2) == 1U;
  controls.deadman = getBits(data, 14, 2) == 1U;
  constexpr unsigned mode_bits[] = {18, 20, 22, 24, 26, 32};
  for (std::uint8_t index = 0; index < 6U; ++index) {
    if (getBits(data, mode_bits[index], 2) == 1U) {
      controls.mode = index + 1U;
    }
  }
  return controls;
}

inline Frame addressClaim(std::uint8_t sa, const Payload & name)
{
  Frame frame;
  frame.priority = 6;
  frame.pf = 0xEE;
  frame.ps = 0xFF;
  frame.sa = sa;
  frame.data = name;
  return frame;
}

inline Payload encodeRemoteAxes(
  std::int16_t joystick_1_x, std::int16_t joystick_1_y,
  std::int16_t joystick_2_x, std::int16_t joystick_2_y)
{
  Payload data{};
  const auto encode_axis = [&data](std::int16_t value, unsigned value_bit, unsigned sign_bit) {
      const auto magnitude = static_cast<std::uint16_t>(
        std::min(1023, std::abs(static_cast<int>(value))));
      setBits(data, value_bit, 10, magnitude);
      setBits(data, sign_bit, 2, value < 0 ? 1U : 0U);
    };
  encode_axis(joystick_1_x, 6, 4);
  encode_axis(joystick_1_y, 22, 20);
  encode_axis(joystick_2_x, 54, 52);
  encode_axis(joystick_2_y, 38, 36);
  return data;
}

inline Payload encodeAkiRemoteControls(
  std::uint8_t mode, bool safety, bool start, bool deadman,
  bool button_plus = false, bool button_minus = false,
  std::int8_t switch_1 = 0, std::int8_t switch_2 = 0)
{
  Payload data{};
  setBits(data, 0, 2, safety ? 1U : 0U);
  setBits(data, 2, 2, start ? 1U : 0U);
  setBits(data, 6, 2, button_plus ? 1U : 0U);
  setBits(data, 8, 2, button_minus ? 1U : 0U);
  setBits(data, 10, 2, switch_1 < 0 ? 1U : 0U);
  setBits(data, 12, 2, switch_1 > 0 ? 1U : 0U);
  setBits(data, 14, 2, deadman ? 1U : 0U);
  constexpr unsigned mode_bits[] = {18, 20, 22, 24, 26, 32};
  if (mode >= 1U && mode <= 6U) {
    setBits(data, mode_bits[mode - 1U], 2, 1U);
  }
  setBits(data, 30, 2, switch_2 > 0 ? 1U : 0U);
  setBits(data, 44, 2, switch_2 < 0 ? 1U : 0U);
  return data;
}

inline double decodeCruiseCommand(const Payload & data)
{
  const auto raw = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[2]) |
    static_cast<std::uint16_t>(data[3]) << 8U);
  if (raw == 0xFFFFU) {
    return 0.0;
  }
  return raw > 32767U ?
    -static_cast<double>(65535U - raw) * 0.001 :
    static_cast<double>(raw) * 0.001;
}

inline double decodeCurvatureCommand(const Payload & data)
{
  const auto raw = static_cast<std::uint16_t>(data[0]) |
    static_cast<std::uint16_t>(data[1]) << 8U;
  return 8032.0 - static_cast<double>(raw) * 0.25;
}

inline Payload encodeWheelSpeed(double speed_m_s, double distance_m, bool key_active)
{
  Payload data{};
  const auto speed_raw = static_cast<std::uint16_t>(
    std::clamp(std::llround(std::abs(speed_m_s) * 1000.0), 0LL, 65534LL));
  const auto distance_raw = static_cast<std::int32_t>(
    std::clamp(std::llround(distance_m * 1000.0),
    static_cast<long long>(INT32_MIN), static_cast<long long>(INT32_MAX)));
  data[0] = static_cast<std::uint8_t>(speed_raw & 0xFFU);
  data[1] = static_cast<std::uint8_t>(speed_raw >> 8U);
  for (unsigned byte = 0; byte < 4; ++byte) {
    data[2 + byte] = static_cast<std::uint8_t>(
      static_cast<std::uint32_t>(distance_raw) >> (8U * byte));
  }
  data[6] = 0xFF;
  const std::uint8_t direction = speed_m_s < 0.0 ? 0U : 1U;
  data[7] = static_cast<std::uint8_t>(direction | (key_active ? 1U << 2U : 0U));
  return data;
}

inline Payload encodeGuidanceStatus(double curvature)
{
  Payload data;
  data.fill(0xFF);
  const auto raw = static_cast<std::uint16_t>(std::clamp(
    std::llround((8032.0 - std::clamp(curvature, -8031.0, 8032.0)) * 4.0),
    0LL, 65535LL));
  data[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  data[1] = static_cast<std::uint8_t>(raw >> 8U);
  data[2] = 0xFD;
  return data;
}

inline Payload encodeCruiseStatus(double speed_m_s)
{
  Payload data;
  data.fill(0xFF);
  data[0] = 0x0A;  // Ground-speed cruise, remote command device.
  const auto raw = static_cast<std::uint16_t>(
    std::clamp(std::llround(std::abs(speed_m_s) * 1000.0), 0LL, 65534LL));
  data[2] = static_cast<std::uint8_t>(raw & 0xFFU);
  data[3] = static_cast<std::uint8_t>(raw >> 8U);
  return data;
}

inline Payload encodeUvcStatus(bool active, double elapsed_s)
{
  Payload data{};
  data[0] = active ? 0x07U : 0x00U;
  const auto time_ds = static_cast<std::uint16_t>(
    std::clamp(std::llround(elapsed_s * 10.0), 0LL, 65535LL));
  data[5] = static_cast<std::uint8_t>(time_ds & 0xFFU);
  data[6] = static_cast<std::uint8_t>(time_ds >> 8U);
  return data;
}

}  // namespace robosoft_simulator::can_protocol

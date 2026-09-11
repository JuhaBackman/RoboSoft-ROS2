// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <array>
#include <cstdint>

namespace robosoft_aki::remote_protocol
{

inline uint32_t getBits(
  const std::array<uint8_t, 8> & data, unsigned start, unsigned length)
{
  uint32_t result = 0;
  for (unsigned bit = 0; bit < length && start + bit < 64; ++bit) {
    const unsigned source = start + bit;
    if ((data[source / 8] & (1U << (source % 8))) != 0U) {
      result |= 1U << bit;
    }
  }
  return result;
}

inline int16_t signedAxis(
  const std::array<uint8_t, 8> & data,
  unsigned value_start, unsigned sign_start)
{
  const auto value = static_cast<int16_t>(getBits(data, value_start, 10));
  return getBits(data, sign_start, 2) == 0 ? value : -value;
}

}  // namespace robosoft_aki::remote_protocol

// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace robosoft_simulator::cola
{
inline void number(std::string & data, uint32_t value, unsigned bytes)
{
  for (unsigned i = bytes; i > 0; --i) {data += char(value >> (8 * (i - 1)));}
}
inline uint32_t read(const std::string & data, size_t offset, unsigned bytes)
{
  uint32_t result = 0;
  for (unsigned i = 0; i < bytes; ++i) {
    result = (result << 8) | static_cast<unsigned char>(data.at(offset + i));
  }
  return result;
}
inline std::string frame(const std::string & payload)
{
  std::string data(4, '\x02');
  number(data, payload.size(), 4);
  data += payload;
  unsigned char checksum = 0;
  for (unsigned char c : payload) {checksum ^= c;}
  data += char(checksum);
  return data;
}
inline std::string text(const std::string & value)
{
  std::string result;
  number(result, value.size(), 2);
  return result + value;
}

// LMDscandata CoLa B: network byte order, millimetres, 1/10000 degree angles.
// Layout follows SICK's sick_lmd_scandata_parser; the driver subtracts 90 deg.
inline std::string scan(const std::vector<uint16_t> & ranges, uint16_t counter,
  uint32_t ticks, bool intensity, bool intensity16, bool polled = false)
{
  std::string p = polled ? "sRA LMDscandata " : "sSN LMDscandata ";
  number(p, 1, 2);  // version
  number(p, 1, 2);  // device number
  number(p, 10000001, 4);  // simulated serial
  number(p, 0, 2);  // device status
  number(p, counter, 2); number(p, counter, 2);
  number(p, ticks, 4); number(p, ticks, 4);
  number(p, 0, 2); number(p, 0, 2); number(p, 0, 2);  // IO, reserved
  number(p, 1500, 4); number(p, 162, 4);  // 15 Hz, 16200 measurements/s
  number(p, 0, 2);  // no encoders
  number(p, intensity && intensity16 ? 2 : 1, 2);
  auto channel = [&](const std::string & name, bool rssi, unsigned bytes) {
      p += name;
      number(p, 0x3f800000, 4); number(p, 0, 4);  // float scale=1, offset=0
      number(p, static_cast<uint32_t>(-450000), 4);
      number(p, 3333, 2); number(p, ranges.size(), 2);
      for (auto range : ranges) {number(p, rssi ? (range ? 100 : 0) : range, bytes);}
    };
  channel("DIST1", false, 2);
  if (intensity && intensity16) {channel("RSSI1", true, 2);}
  number(p, intensity && !intensity16 ? 1 : 0, 2);
  if (intensity && !intensity16) {channel("RSSI1", true, 1);}
  for (int i = 0; i < 5; ++i) {number(p, 0, 2);}  // position/name/comment/time/event
  return p;
}
}  // namespace robosoft_simulator::cola

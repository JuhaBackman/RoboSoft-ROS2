// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robosoft_core
{

struct Nmea0183Fix
{
  double latitude{0.0};
  double longitude{0.0};
  double altitude{0.0};
  int fix_quality{0};
  int satellites{0};
  double hdop{0.0};
};

struct Nmea0183Uncertainty
{
  double rms{0.0};
  double latitude_stddev{0.0};
  double longitude_stddev{0.0};
  double altitude_stddev{0.0};
};

struct Nmea0183Velocity
{
  double compass_rad{0.0};
  double speed_m_s{0.0};
};

struct Nmea0183Attitude
{
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

struct Nmea0183Update
{
  std::optional<Nmea0183Fix> fix;
  std::optional<Nmea0183Uncertainty> uncertainty;
  std::optional<Nmea0183Velocity> velocity;
  std::optional<Nmea0183Attitude> attitude;
};

/// Parses supported standard and KindHelm-specific NMEA sentences.
class Nmea0183Parser
{
public:
  std::optional<Nmea0183Update> parse(const std::string & sentence) const;

private:
  static bool validChecksum(const std::string & sentence);
  static std::vector<std::string> fields(const std::string & sentence);
  static std::optional<double> number(const std::string & value);
  static std::optional<int> integer(const std::string & value);
  static std::optional<double> coordinate(
    const std::string & value, const std::string & hemisphere);
  static double wrapAngle(double angle);
};

}  // namespace robosoft_core

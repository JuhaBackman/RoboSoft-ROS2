// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace robosoft_simulator::nmea0183
{

// Must match the local equirectangular projection in robosoft_core. This makes
// simulated local odometry and the GNSS round trip exact in the map frame.
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

inline double wrapDegrees(double degrees)
{
  while (degrees >= 360.0) degrees -= 360.0;
  while (degrees < 0.0) degrees += 360.0;
  return degrees;
}

inline std::string sentence(const std::string & payload)
{
  std::uint8_t checksum = 0U;
  for (const unsigned char character : payload) checksum ^= character;
  std::ostringstream output;
  output << '$' << payload << '*' << std::uppercase << std::hex << std::setw(2) <<
    std::setfill('0') << static_cast<unsigned>(checksum) << "\r\n";
  return output.str();
}

inline std::string coordinate(double degrees, bool latitude)
{
  const double absolute = std::abs(degrees);
  const int whole_degrees = static_cast<int>(std::floor(absolute));
  const double minutes = (absolute - whole_degrees) * 60.0;
  std::ostringstream output;
  output << std::setfill('0') << std::setw(latitude ? 2 : 3) << whole_degrees <<
    std::fixed << std::setprecision(7) << std::setw(10) << minutes;
  return output.str();
}

inline double latitudeFromNorthing(double reference_latitude, double northing_m)
{
  return reference_latitude + northing_m / kEarthRadiusM * 180.0 / kPi;
}

inline double longitudeFromEasting(
  double reference_latitude, double reference_longitude, double easting_m)
{
  return reference_longitude + easting_m /
    (kEarthRadiusM * std::cos(reference_latitude * kPi / 180.0)) * 180.0 / kPi;
}

}  // namespace robosoft_simulator::nmea0183

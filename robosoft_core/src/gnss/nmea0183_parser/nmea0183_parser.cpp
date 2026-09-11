// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "nmea0183_parser.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace robosoft_core
{

namespace
{
constexpr double degrees_to_radians = M_PI / 180.0;
}

bool Nmea0183Parser::validChecksum(const std::string & sentence)
{
  const auto star = sentence.find('*');
  if (sentence.empty() || sentence.front() != '$' ||
    star == std::string::npos || star + 2 >= sentence.size())
  {
    return false;
  }
  uint8_t checksum = 0;
  for (std::size_t index = 1; index < star; ++index) {
    checksum ^= static_cast<uint8_t>(sentence[index]);
  }
  char * end = nullptr;
  const auto expected = std::strtoul(sentence.substr(star + 1, 2).c_str(), &end, 16);
  return end != nullptr && *end == '\0' && checksum == expected;
}

std::vector<std::string> Nmea0183Parser::fields(const std::string & sentence)
{
  const auto end = sentence.find('*');
  std::stringstream stream(sentence.substr(1, end - 1));
  std::vector<std::string> result;
  for (std::string field; std::getline(stream, field, ','); ) {
    const auto first = field.find_first_not_of(" \t\r\n");
    const auto last = field.find_last_not_of(" \t\r\n");
    result.push_back(
      first == std::string::npos ? "" : field.substr(first, last - first + 1));
  }
  return result;
}

std::optional<double> Nmea0183Parser::number(const std::string & value)
{
  if (value.empty()) {
    return std::nullopt;
  }
  char * end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  return end != value.c_str() && *end == '\0' && std::isfinite(parsed) ?
         std::optional<double>(parsed) : std::nullopt;
}

std::optional<int> Nmea0183Parser::integer(const std::string & value)
{
  const auto parsed = number(value);
  return parsed ? std::optional<int>(static_cast<int>(*parsed)) : std::nullopt;
}

std::optional<double> Nmea0183Parser::coordinate(
  const std::string & value, const std::string & hemisphere)
{
  const auto raw = number(value);
  if (!raw || hemisphere.empty()) {
    return std::nullopt;
  }
  const double degrees = std::floor(*raw / 100.0);
  double coordinate = degrees + (*raw - degrees * 100.0) / 60.0;
  if (hemisphere == "S" || hemisphere == "W") {
    coordinate = -coordinate;
  } else if (hemisphere != "N" && hemisphere != "E") {
    return std::nullopt;
  }
  return coordinate;
}

double Nmea0183Parser::wrapAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}

std::optional<Nmea0183Update> Nmea0183Parser::parse(
  const std::string & sentence) const
{
  if (!validChecksum(sentence)) {
    return std::nullopt;
  }
  const auto values = fields(sentence);
  if (values.empty()) {
    return std::nullopt;
  }

  Nmea0183Update update;
  const std::string type =
    values[0].size() >= 3 ? values[0].substr(values[0].size() - 3) : values[0];
  if (type == "GGA" && values.size() >= 10) {
    const auto latitude = coordinate(values[2], values[3]);
    const auto longitude = coordinate(values[4], values[5]);
    const auto quality = integer(values[6]);
    const auto satellites = integer(values[7]);
    const auto hdop = number(values[8]);
    const auto altitude = number(values[9]);
    if (!latitude || !longitude || !quality || !satellites || !hdop || !altitude) {
      return std::nullopt;
    }
    update.fix = Nmea0183Fix{
      *latitude, *longitude, *altitude, *quality, *satellites, *hdop};
  } else if (type == "GST" && values.size() >= 9) {
    const auto rms = number(values[2]);
    const auto latitude = number(values[6]);
    const auto longitude = number(values[7]);
    const auto altitude = number(values[8]);
    if (!rms || !latitude || !longitude || !altitude) {
      return std::nullopt;
    }
    update.uncertainty = Nmea0183Uncertainty{
      *rms, *latitude, *longitude, *altitude};
  } else if (type == "VTG" && values.size() >= 9) {
    const auto track = number(values[1]);
    const auto speed_kph = number(values[7]);
    if (!track || !speed_kph) {
      return std::nullopt;
    }
    update.velocity = Nmea0183Velocity{
      *track * degrees_to_radians, *speed_kph / 3.6};
  } else if (values[0] == "PKHM" && values.size() >= 8) {
    const auto yaw = number(values[5]);
    const auto pitch = number(values[6]);
    const auto roll = number(values[7]);
    if (!yaw || !pitch || !roll) {
      return std::nullopt;
    }
    update.attitude = Nmea0183Attitude{
      wrapAngle(*roll * degrees_to_radians),
      wrapAngle(*pitch * degrees_to_radians),
      wrapAngle(-*yaw * degrees_to_radians + M_PI / 2.0)};
  } else {
    return std::nullopt;
  }
  return update;
}

}  // namespace robosoft_core

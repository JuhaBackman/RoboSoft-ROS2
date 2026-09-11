// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinyxml2.h>

#include "robosoft_simulator/nmea0183.hpp"

namespace robosoft_simulator::lidar
{

struct Pole
{
  double x;
  double y;
};

/// Load direct PFD/PNT type-5 point obstacles from an ISO 11783 TASK file.
inline std::vector<Pole> loadPoles(
  const std::string & file, double latitude_origin, double longitude_origin)
{
  tinyxml2::XMLDocument document;
  if (document.LoadFile(file.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Cannot read lidar TASK: " + file);
  }
  auto root = document.FirstChildElement("ISO11783_TaskData");
  if (!root) {
    throw std::runtime_error("Missing ISO11783_TaskData in " + file);
  }
  std::vector<Pole> poles;
  for (auto field = root->FirstChildElement("PFD"); field;
    field = field->NextSiblingElement("PFD"))
  {
    for (auto point = field->FirstChildElement("PNT"); point;
      point = point->NextSiblingElement("PNT"))
    {
      if (point->IntAttribute("A") != 5) {
        continue;
      }
      double latitude;
      double longitude;
      if (point->QueryDoubleAttribute("C", &latitude) != tinyxml2::XML_SUCCESS ||
        point->QueryDoubleAttribute("D", &longitude) != tinyxml2::XML_SUCCESS ||
        !std::isfinite(latitude) || !std::isfinite(longitude) ||
        std::abs(latitude) > 90.0 || std::abs(longitude) > 180.0)
      {
        throw std::runtime_error("Invalid obstacle C/D coordinates in " + file);
      }
      const double scale = nmea0183::kEarthRadiusM * nmea0183::kPi / 180.0;
      poles.push_back({
        (longitude - longitude_origin) * scale *
        std::cos(latitude_origin * nmea0183::kPi / 180.0),
        (latitude - latitude_origin) * scale});
    }
  }
  return poles;
}

/// Ray-cast the nearest surface of each circular pole into a TiM5xx scan.
inline std::vector<uint16_t> ranges(
  const std::vector<Pole> & poles, double x, double y, double yaw,
  double radius, double min_range, double max_range)
{
  std::vector<uint16_t> result(811, 0);
  for (std::size_t index = 0; index < result.size(); ++index) {
    const double angle =
      yaw + (-135.0 + static_cast<double>(index) * 0.3333) *
      nmea0183::kPi / 180.0;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    double nearest = max_range;
    bool hit = false;
    for (const auto & pole : poles) {
      const double px = pole.x - x;
      const double py = pole.y - y;
      const double along = px * dx + py * dy;
      const double across = px * dy - py * dx;
      const double discriminant = radius * radius - across * across;
      if (discriminant < 0.0) {
        continue;
      }
      double distance = along - std::sqrt(discriminant);
      if (distance < 0.0) {
        distance = along + std::sqrt(discriminant);
      }
      if (distance >= 0.0 && distance <= nearest) {
        nearest = distance;
        hit = true;
      }
    }
    if (hit && nearest >= min_range) {
      result[index] = static_cast<uint16_t>(std::lround(nearest * 1000.0));
    }
  }
  return result;
}

}  // namespace robosoft_simulator::lidar

// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "robosoft_simulator/nmea0183.hpp"

#include <gtest/gtest.h>

TEST(Nmea0183Simulator, AddsStandardChecksumAndLineEnding)
{
  EXPECT_EQ(
    robosoft_simulator::nmea0183::sentence("GPVTG,90.0,T,,M,,N,3.6,K"),
    "$GPVTG,90.0,T,,M,,N,3.6,K*72\r\n");
}

TEST(Nmea0183Simulator, ConvertsLocalEnuToWgs84)
{
  constexpr double latitude = 60.0;
  constexpr double longitude = 25.0;
  EXPECT_NEAR(
    robosoft_simulator::nmea0183::latitudeFromNorthing(latitude, 100.0),
    60.000899322, 1e-8);
  EXPECT_NEAR(
    robosoft_simulator::nmea0183::longitudeFromEasting(latitude, longitude, 100.0),
    25.001798643, 1e-8);
}

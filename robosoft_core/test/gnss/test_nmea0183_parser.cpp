// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "nmea0183_parser.hpp"

TEST(Nmea0183Parser, ParsesGga)
{
  robosoft_core::Nmea0183Parser parser;
  const auto update = parser.parse(
    "$GNGGA,103051.19,6142.95264275,N,02717.83450885,E,5,32,0.60,"
    "86.647,M,17.822,M,1.0,1912*5F");

  ASSERT_TRUE(update);
  ASSERT_TRUE(update->fix);
  EXPECT_NEAR(update->fix->latitude, 61.7158773792, 1e-9);
  EXPECT_NEAR(update->fix->longitude, 27.2972418142, 1e-9);
  EXPECT_EQ(update->fix->fix_quality, 5);
}

TEST(Nmea0183Parser, ConvertsPkhmAttitudeToRosConvention)
{
  robosoft_core::Nmea0183Parser parser;
  const auto update = parser.parse(
    "$PKHM,103051.19,21,04,2023, 359.638153, 1.459643, 267.194519*27");

  ASSERT_TRUE(update);
  ASSERT_TRUE(update->attitude);
  EXPECT_NEAR(update->attitude->yaw, 1.577112, 1e-5);
  EXPECT_NEAR(update->attitude->pitch, 0.0254756, 1e-5);
  EXPECT_NEAR(update->attitude->roll, -1.6197612, 1e-6);
}

TEST(Nmea0183Parser, RejectsBadChecksum)
{
  robosoft_core::Nmea0183Parser parser;
  EXPECT_FALSE(parser.parse("$GNVTG,0.00,T,10.82,M,0.00,N,0.00,K,F*00"));
}

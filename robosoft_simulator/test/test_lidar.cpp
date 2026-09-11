// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only
#include "lidar_model.hpp"
#include "cola.hpp"
#include <gtest/gtest.h>
#include <unistd.h>
#include <cstdio>

TEST(Lidar, LoadsOnlyPartfieldObstaclePointsAndProjectsCoordinates)
{
  char name[] = "/tmp/robosoft-lidar-xml-XXXXXX";
  const int fd = mkstemp(name);
  ASSERT_GE(fd, 0);
  FILE * file = fdopen(fd, "w");
  ASSERT_NE(file, nullptr);
  fputs("<ISO11783_TaskData><PFD>"
    "<PNT A='5' C='60' D='25.0001'/>"
    "<PNT A='1' C='60' D='25'/>"
    "<PLN><PNT A='5' C='60' D='25'/></PLN>"
    "</PFD><TSK><PNT A='5' C='60' D='25'/></TSK></ISO11783_TaskData>", file);
  fclose(file);
  const auto poles = robosoft_simulator::lidar::loadPoles(name, 60, 25);
  unlink(name);
  ASSERT_EQ(poles.size(), 1u);
  EXPECT_NEAR(poles[0].x, 5.559746, 0.00001);
  EXPECT_NEAR(poles[0].y, 0, 0.00001);
}

TEST(Lidar, IntersectsSurfaceAndOccludesFurtherPoles)
{
  using robosoft_simulator::lidar::ranges;
  auto scan = ranges({{5, 0}, {10, 0}}, 0, 0, 0, 0.035, 0.05, 25);
  EXPECT_NEAR(scan.at(405), 4965, 1);
  EXPECT_EQ(scan.at(0), 0);
  EXPECT_EQ(scan.at(810), 0);
  // Turning the scanner north removes the eastward pole from its centre ray.
  scan = ranges({{5, 0}}, 0, 0, robosoft_simulator::nmea0183::kPi / 2, 0.035, 0.05, 25);
  EXPECT_EQ(scan.at(405), 0);
  scan = ranges({{5, 0}}, 1, 0, 0, 0.035, 0.05, 25);
  EXPECT_NEAR(scan.at(405), 3965, 1);
}

TEST(Lidar, BinaryScanHasCorrectUnitsAndOffsets)
{
  using namespace robosoft_simulator;
  const auto packet = cola::frame(cola::scan(std::vector<uint16_t>(811, 1234), 7,
    123456, true, true));
  EXPECT_EQ(cola::read(packet, 38, 4), 123456u);
  EXPECT_EQ(cola::read(packet, 52, 4), 1500u);
  EXPECT_EQ(cola::read(packet, 62, 2), 2u);
  EXPECT_EQ(packet.substr(64, 5), "DIST1");
  EXPECT_EQ(cola::read(packet, 83, 2), 811u);
  EXPECT_EQ(cola::read(packet, 85, 2), 1234u);
  EXPECT_EQ(cola::read(packet, 4, 4), packet.size() - 9);
}

// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "remote_protocol.hpp"

TEST(RemoteProtocol, ExtractsTenBitsAcrossByteBoundary)
{
  std::array<uint8_t, 8> data{};
  const uint16_t expected = 0x2A5;
  for (unsigned bit = 0; bit < 10; ++bit) {
    if ((expected & (1U << bit)) != 0U) {
      const unsigned target = 6 + bit;
      data[target / 8] |= static_cast<uint8_t>(1U << (target % 8));
    }
  }
  EXPECT_EQ(robosoft_aki::remote_protocol::getBits(data, 6, 10),
            expected);
}

TEST(RemoteProtocol, AppliesConfiguredAxisSign)
{
  std::array<uint8_t, 8> data{};
  const uint16_t magnitude = 500;
  for (unsigned bit = 0; bit < 10; ++bit) {
    if ((magnitude & (1U << bit)) != 0U) {
      const unsigned target = 6 + bit;
      data[target / 8] |= static_cast<uint8_t>(1U << (target % 8));
    }
  }
  data[0] |= static_cast<uint8_t>(1U << 4);
  EXPECT_EQ(robosoft_aki::remote_protocol::signedAxis(data, 6, 4), -500);
}

TEST(RemoteProtocol, DoesNotReadPastPayload)
{
  std::array<uint8_t, 8> data{};
  data[7] = 0xF0;
  EXPECT_EQ(robosoft_aki::remote_protocol::getBits(data, 60, 16), 0x0F);
}

// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <gtest/gtest.h>

#include "ntrip_protocol.hpp"

TEST(NtripProtocol, BuildsAuthenticatedRequestWithGga)
{
  const auto value = robosoft_core::ntrip::request(
    "caster.example", 2101, "MOUNT", "user", "pass", "$GNGGA,1*00");
  EXPECT_NE(value.find("GET /MOUNT HTTP/1.1\r\n"), std::string::npos);
  EXPECT_NE(value.find("Authorization: Basic dXNlcjpwYXNz\r\n"), std::string::npos);
  EXPECT_NE(value.find("Ntrip-GGA: $GNGGA,1*00\r\n"), std::string::npos);
}

TEST(NtripProtocol, AcceptsIcyAndHttpResponses)
{
  const auto icy = robosoft_core::ntrip::parseResponse(
    "ICY 200 OK\r\nContent-Type: gnss/data\r\n\r\nabc");
  ASSERT_TRUE(icy);
  EXPECT_TRUE(icy->accepted);
  EXPECT_FALSE(icy->chunked);

  const auto http = robosoft_core::ntrip::parseResponse(
    "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n");
  ASSERT_TRUE(http);
  EXPECT_TRUE(http->accepted);
  EXPECT_TRUE(http->chunked);
}

TEST(NtripProtocol, BuildsNtripVersionOneRequest)
{
  const auto value = robosoft_core::ntrip::request(
    "caster.example", 2101, "MOUNT", "", "", "", false);
  EXPECT_NE(value.find("GET /MOUNT HTTP/1.0\r\n"), std::string::npos);
  EXPECT_EQ(value.find("Ntrip-Version:"), std::string::npos);
}

TEST(NtripProtocol, DecodesSplitChunkedStream)
{
  robosoft_core::ntrip::ChunkDecoder decoder;
  const std::string first = "3\r\n\x01";
  const std::string second = "\x02\x03\r\n2\r\n\x04\x05\r\n0\r\n\r\n";
  const auto output1 = decoder.append(
    reinterpret_cast<const uint8_t *>(first.data()), first.size());
  ASSERT_TRUE(output1);
  EXPECT_TRUE(output1->empty());
  const auto output2 = decoder.append(
    reinterpret_cast<const uint8_t *>(second.data()), second.size());
  ASSERT_TRUE(output2);
  EXPECT_EQ(*output2, (std::vector<uint8_t>{1, 2, 3, 4, 5}));
  EXPECT_TRUE(decoder.complete());
}

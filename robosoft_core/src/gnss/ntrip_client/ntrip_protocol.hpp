// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace robosoft_core::ntrip
{

inline std::string base64(const std::string & input)
{
  static constexpr char alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(((input.size() + 2U) / 3U) * 4U);
  for (std::size_t index = 0; index < input.size(); index += 3U) {
    const auto first = static_cast<uint8_t>(input[index]);
    const auto second = index + 1U < input.size() ?
      static_cast<uint8_t>(input[index + 1U]) : 0U;
    const auto third = index + 2U < input.size() ?
      static_cast<uint8_t>(input[index + 2U]) : 0U;
    const uint32_t value =
      (static_cast<uint32_t>(first) << 16U) |
      (static_cast<uint32_t>(second) << 8U) | third;
    output.push_back(alphabet[(value >> 18U) & 0x3fU]);
    output.push_back(alphabet[(value >> 12U) & 0x3fU]);
    output.push_back(index + 1U < input.size() ?
      alphabet[(value >> 6U) & 0x3fU] : '=');
    output.push_back(index + 2U < input.size() ? alphabet[value & 0x3fU] : '=');
  }
  return output;
}

inline std::string request(
  const std::string & server, int port, const std::string & mountpoint,
  const std::string & username, const std::string & password,
  const std::string & gga, bool ntrip2 = true)
{
  std::ostringstream stream;
  stream << "GET /" << mountpoint << (ntrip2 ? " HTTP/1.1\r\n" : " HTTP/1.0\r\n");
  if (ntrip2) {
    stream << "Host: " << server << ':' << port << "\r\n"
           << "Ntrip-Version: Ntrip/2.0\r\n";
  }
  stream << "User-Agent: NTRIP RoboSoft-ROS2/0.1\r\n"
         << "Accept: */*\r\n"
         << "Connection: close\r\n";
  if (!username.empty() || !password.empty()) {
    stream << "Authorization: Basic " << base64(username + ':' + password) << "\r\n";
  }
  if (!gga.empty()) {
    stream << "Ntrip-GGA: " << gga << "\r\n";
  }
  stream << "\r\n";
  return stream.str();
}

struct Response
{
  bool accepted{false};
  bool chunked{false};
  std::size_t body_offset{0};
  std::string status_line;
};

inline std::optional<Response> parseResponse(const std::string & data)
{
  const auto header_end = data.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    return std::nullopt;
  }
  const auto line_end = data.find("\r\n");
  Response response;
  response.status_line = data.substr(0, line_end);
  response.accepted =
    response.status_line.rfind("ICY 200", 0) == 0 ||
    response.status_line.rfind("HTTP/1.0 200", 0) == 0 ||
    response.status_line.rfind("HTTP/1.1 200", 0) == 0;
  std::string headers = data.substr(0, header_end);
  std::transform(
    headers.begin(), headers.end(), headers.begin(),
    [](unsigned char character) {return static_cast<char>(std::tolower(character));});
  response.chunked = headers.find("transfer-encoding: chunked") != std::string::npos;
  response.body_offset = header_end + 4U;
  return response;
}

class ChunkDecoder
{
public:
  std::optional<std::vector<uint8_t>> append(const uint8_t * data, std::size_t size)
  {
    buffer_.append(reinterpret_cast<const char *>(data), size);
    std::vector<uint8_t> output;
    for (;;) {
      if (remaining_ == 0U) {
        const auto line_end = buffer_.find("\r\n");
        if (line_end == std::string::npos) {
          break;
        }
        const auto extension = buffer_.find(';');
        const auto count_end = extension < line_end ? extension : line_end;
        try {
          remaining_ = std::stoul(buffer_.substr(0, count_end), nullptr, 16);
        } catch (...) {
          return std::nullopt;
        }
        buffer_.erase(0, line_end + 2U);
        if (remaining_ == 0U) {
          complete_ = true;
          break;
        }
      }
      if (buffer_.size() < remaining_ + 2U) {
        break;
      }
      output.insert(output.end(), buffer_.begin(), buffer_.begin() + remaining_);
      if (buffer_[remaining_] != '\r' || buffer_[remaining_ + 1U] != '\n') {
        return std::nullopt;
      }
      buffer_.erase(0, remaining_ + 2U);
      remaining_ = 0U;
    }
    return output;
  }

  bool complete() const {return complete_;}

private:
  std::string buffer_;
  std::size_t remaining_{0};
  bool complete_{false};
};

}  // namespace robosoft_core::ntrip

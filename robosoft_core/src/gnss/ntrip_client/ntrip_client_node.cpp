// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "ntrip_protocol.hpp"
#include "robosoft_interfaces/topics.hpp"

namespace robosoft_core
{

class NtripClientNode final : public rclcpp::Node
{
public:
  explicit NtripClientNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ntrip_client_node", options)
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    server_ = declare_parameter<std::string>("server", "");
    port_ = declare_parameter<int>("port", 2101);
    mountpoint_ = declare_parameter<std::string>("mountpoint", "");
    protocol_version_ = declare_parameter<std::string>("protocol_version", "auto");
    username_ = declare_parameter<std::string>("username", "");
    password_ = declare_parameter<std::string>("password", "");
    require_gga_ = declare_parameter<bool>("require_gga", true);
    gga_interval_s_ = declare_parameter<double>("gga_interval_s", 5.0);
    reconnect_delay_s_ = declare_parameter<double>("reconnect_delay_s", 10.0);
    connect_timeout_s_ = declare_parameter<double>("connect_timeout_s", 10.0);

    correction_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
      robosoft_interfaces::kNtripCorrectionsTopic, rclcpp::SensorDataQoS());
    sentence_sub_ = create_subscription<std_msgs::msg::String>(
      robosoft_interfaces::kNmea0183SentenceTopic, rclcpp::QoS(100),
      std::bind(&NtripClientNode::onSentence, this, std::placeholders::_1));

    if (!enabled_) {
      RCLCPP_WARN(get_logger(), "NTRIP client is disabled by configuration");
      return;
    }
    if (server_.empty() || mountpoint_.empty()) {
      RCLCPP_ERROR(get_logger(), "NTRIP server and mountpoint are required");
      return;
    }
    if (protocol_version_ != "auto" && protocol_version_ != "1" &&
      protocol_version_ != "2")
    {
      RCLCPP_ERROR(get_logger(), "protocol_version must be auto, 1 or 2");
      return;
    }
    use_ntrip2_ = protocol_version_ != "1";
    running_ = true;
    worker_ = std::thread(&NtripClientNode::run, this);
  }

  ~NtripClientNode() override
  {
    running_ = false;
    const int socket = socket_fd_.exchange(-1);
    if (socket >= 0) {
      ::shutdown(socket, SHUT_RDWR);
      ::close(socket);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void onSentence(const std_msgs::msg::String & message)
  {
    if (message.data.rfind("$GNGGA,", 0) != 0 &&
      message.data.rfind("$GPGGA,", 0) != 0)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(gga_mutex_);
    latest_gga_ = message.data;
  }

  std::string gga() const
  {
    std::lock_guard<std::mutex> lock(gga_mutex_);
    return latest_gga_;
  }

  int connectTcp()
  {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * addresses = nullptr;
    const std::string port = std::to_string(port_);
    const int lookup = ::getaddrinfo(server_.c_str(), port.c_str(), &hints, &addresses);
    if (lookup != 0) {
      RCLCPP_ERROR(get_logger(), "NTRIP address lookup failed: %s", gai_strerror(lookup));
      return -1;
    }

    int connected = -1;
    for (auto * address = addresses; address && running_; address = address->ai_next) {
      const int socket = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
      if (socket < 0) {
        continue;
      }
      const int flags = ::fcntl(socket, F_GETFL, 0);
      ::fcntl(socket, F_SETFL, flags | O_NONBLOCK);
      const int result = ::connect(socket, address->ai_addr, address->ai_addrlen);
      if (result == 0 || errno == EINPROGRESS) {
        pollfd descriptor{socket, POLLOUT, 0};
        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(connect_timeout_s_);
        while (running_ && std::chrono::steady_clock::now() < deadline) {
          if (::poll(&descriptor, 1, 200) > 0) {
            int error = 0;
            socklen_t length = sizeof(error);
            if (::getsockopt(socket, SOL_SOCKET, SO_ERROR, &error, &length) == 0 &&
              error == 0)
            {
              ::fcntl(socket, F_SETFL, flags | O_NONBLOCK);
              connected = socket;
            }
            break;
          }
        }
        if (connected >= 0) {
          break;
        }
      }
      ::close(socket);
    }
    ::freeaddrinfo(addresses);
    return connected;
  }

  bool sendAll(int socket, const std::string & data)
  {
    std::size_t sent = 0;
    while (running_ && sent < data.size()) {
      const ssize_t count = ::send(socket, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
      if (count > 0) {
        sent += static_cast<std::size_t>(count);
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        pollfd descriptor{socket, POLLOUT, 0};
        if (::poll(&descriptor, 1, 1000) < 0 && errno != EINTR) {
          return false;
        }
      } else {
        return false;
      }
    }
    return sent == data.size();
  }

  void deliver(const uint8_t * data, std::size_t size)
  {
    if (size == 0U) {
      return;
    }
    std_msgs::msg::UInt8MultiArray message;
    message.data.assign(data, data + size);
    correction_pub_->publish(message);
  }

  bool stream(int socket)
  {
    const std::string current_gga = gga();
    if (!sendAll(
        socket, ntrip::request(
          server_, port_, mountpoint_, username_, password_, current_gga,
          use_ntrip2_)))
    {
      return false;
    }

    std::string header;
    header.reserve(4096);
    std::vector<uint8_t> pending_body;
    ntrip::Response response;
    for (;;) {
      uint8_t data[4096];
      const ssize_t count = ::recv(socket, data, sizeof(data), 0);
      if (count > 0) {
        header.append(reinterpret_cast<const char *>(data), static_cast<std::size_t>(count));
        const auto parsed = ntrip::parseResponse(header);
        if (!parsed) {
          if (header.size() > 16384U) {
            RCLCPP_ERROR(get_logger(), "NTRIP response header is too large");
            return false;
          }
          continue;
        }
        response = *parsed;
        if (!response.accepted) {
          RCLCPP_ERROR(
            get_logger(), "NTRIP caster rejected the request: %s",
            response.status_line.c_str());
          if (protocol_version_ == "auto" && use_ntrip2_) {
            use_ntrip2_ = false;
            RCLCPP_WARN(get_logger(), "Falling back to NTRIP v1 on the next connection");
          }
          return false;
        }
        pending_body.assign(
          header.begin() + static_cast<std::ptrdiff_t>(response.body_offset), header.end());
        break;
      }
      if (count == 0) {
        return false;
      }
      if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        return false;
      }
      pollfd descriptor{socket, POLLIN, 0};
      if (::poll(&descriptor, 1, 1000) < 0 && errno != EINTR) {
        return false;
      }
      if (!running_) {
        return false;
      }
    }

    RCLCPP_INFO(
      get_logger(), "NTRIP v%d correction stream connected", use_ntrip2_ ? 2 : 1);
    ntrip::ChunkDecoder chunks;
    auto process = [this, &response, &chunks](const uint8_t * data, std::size_t size) {
        if (!response.chunked) {
          deliver(data, size);
          return true;
        }
        const auto decoded = chunks.append(data, size);
        if (!decoded) {
          RCLCPP_ERROR(get_logger(), "Invalid NTRIP chunked transfer encoding");
          return false;
        }
        deliver(decoded->data(), decoded->size());
        return !chunks.complete();
      };
    if (!pending_body.empty() && !process(pending_body.data(), pending_body.size())) {
      return chunks.complete();
    }

    auto next_gga = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(gga_interval_s_);
    while (running_) {
      pollfd descriptor{socket, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 500);
      if (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        return false;
      }
      if (ready > 0 && (descriptor.revents & POLLIN)) {
        uint8_t data[4096];
        const ssize_t count = ::recv(socket, data, sizeof(data), 0);
        if (count <= 0) {
          return false;
        }
        if (!process(data, static_cast<std::size_t>(count))) {
          return chunks.complete();
        }
      } else if (ready < 0 && errno != EINTR) {
        return false;
      }

      if (std::chrono::steady_clock::now() >= next_gga) {
        const auto current = gga();
        if (!current.empty() && !sendAll(socket, current + "\r\n")) {
          return false;
        }
        next_gga = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(gga_interval_s_);
      }
    }
    return true;
  }

  void run()
  {
    while (running_) {
      if (require_gga_ && gga().empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }
      const int socket = connectTcp();
      if (socket >= 0) {
        socket_fd_ = socket;
        stream(socket);
        if (socket_fd_.exchange(-1) == socket) {
          ::close(socket);
        }
        RCLCPP_WARN(get_logger(), "NTRIP correction stream disconnected");
      } else {
        RCLCPP_WARN(get_logger(), "Cannot connect to NTRIP caster %s:%d", server_.c_str(), port_);
      }
      const auto delay = std::chrono::duration<double>(reconnect_delay_s_);
      const auto end = std::chrono::steady_clock::now() + delay;
      while (running_ && std::chrono::steady_clock::now() < end) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  bool enabled_{false};
  bool require_gga_{true};
  std::string server_;
  std::string mountpoint_;
  std::string protocol_version_;
  std::string username_;
  std::string password_;
  int port_{2101};
  double gga_interval_s_{5.0};
  double reconnect_delay_s_{10.0};
  double connect_timeout_s_{10.0};
  bool use_ntrip2_{true};
  mutable std::mutex gga_mutex_;
  std::string latest_gga_;
  std::atomic<bool> running_{false};
  std::atomic<int> socket_fd_{-1};
  std::thread worker_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sentence_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr correction_pub_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::NtripClientNode>());
  rclcpp::shutdown();
  return 0;
}

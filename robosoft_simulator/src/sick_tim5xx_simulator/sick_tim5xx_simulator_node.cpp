// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "cola.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace robosoft_simulator
{

/// Generic PointCloud2-to-SICK TiM5xx Ethernet protocol adapter.
class SickTim5xxSimulator final : public rclcpp::Node
{
public:
  explicit SickTim5xxSimulator(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sick_tim5xx_simulator_node", options), ranges_(kRayCount, 0)
  {
    const auto cloud_topic =
      declare_parameter<std::string>("cloud_topic", "cloud");
    cloud_timeout_ = declare_parameter<double>("cloud_timeout_s", 1.0);
    const double scan_rate = declare_parameter<double>("scan_rate_hz", 15.0);
    if (!std::isfinite(cloud_timeout_) || cloud_timeout_ <= 0.0 ||
      !std::isfinite(scan_rate) || scan_rate <= 0.0)
    {
      throw std::runtime_error("Invalid cloud timeout or scan rate");
    }

    const auto address =
      declare_parameter<std::string>("bind_address", "0.0.0.0");
    const int port = declare_parameter<int>("port", 2112);
    openListener(address, port);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&SickTim5xxSimulator::onCloud, this, std::placeholders::_1));
    io_timer_ = create_wall_timer(
      std::chrono::milliseconds(5), [this]() {pollSocket();});
    scan_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / scan_rate), [this]() {
        if (client_ >= 0 && streaming_ && measuring_ && cloudFresh()) {
          queue(cola::scan(ranges_, ++counter_, ticks(), intensity_, intensity16_));
        }
      });
    RCLCPP_INFO(
      get_logger(), "Generic TiM5xx simulator listening on %s:%d; input '%s'",
      address.c_str(), port, cloud_topic.c_str());
  }

  ~SickTim5xxSimulator() override
  {
    disconnect();
    if (listener_ >= 0) {
      close(listener_);
    }
  }

private:
  using Clock = std::chrono::steady_clock;
  static constexpr std::size_t kRayCount = 811;
  static constexpr double kStartAngleRad = -135.0 * M_PI / 180.0;
  static constexpr double kAngleStepRad = 0.3333 * M_PI / 180.0;

  void openListener(const std::string & address, int port)
  {
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    if (port < 1 || port > 65535 ||
      inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1)
    {
      throw std::runtime_error("Invalid lidar IPv4 address or TCP port");
    }
    endpoint.sin_port = htons(port);
    listener_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    int reuse = 1;
    if (listener_ >= 0) {
      setsockopt(listener_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    }
    if (listener_ < 0 ||
      bind(listener_, reinterpret_cast<sockaddr *>(&endpoint), sizeof(endpoint)) < 0 ||
      listen(listener_, 1) < 0)
    {
      const std::string error = std::strerror(errno);
      if (listener_ >= 0) {
        close(listener_);
        listener_ = -1;
      }
      throw std::runtime_error("Cannot listen for lidar: " + error);
    }
  }

  void onCloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    std::vector<uint16_t> converted(kRayCount, 0);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      for (; x != x.end(); ++x, ++y) {
        if (!std::isfinite(*x) || !std::isfinite(*y)) {
          continue;
        }
        const double range = std::hypot(*x, *y);
        const double ray =
          (std::atan2(*y, *x) - kStartAngleRad) / kAngleStepRad;
        const long index = std::lround(ray);
        if (index < 0 || index >= static_cast<long>(kRayCount) ||
          std::abs(ray - static_cast<double>(index)) > 0.51 ||
          range <= 0.0 || range > 65.535)
        {
          continue;
        }
        const auto millimetres =
          static_cast<uint16_t>(std::lround(range * 1000.0));
        auto & target = converted[static_cast<std::size_t>(index)];
        if (target == 0 || millimetres < target) {
          target = millimetres;
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud has no usable x/y fields: %s", error.what());
      return;
    }
    ranges_ = std::move(converted);
    cloud_received_ = true;
    last_cloud_ = Clock::now();
  }

  bool cloudFresh() const
  {
    return cloud_received_ &&
           std::chrono::duration<double>(Clock::now() - last_cloud_).count() <
           cloud_timeout_;
  }

  uint32_t ticks() const
  {
    return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - boot_)
      .count());
  }

  void disconnect()
  {
    if (client_ >= 0) {
      close(client_);
      client_ = -1;
    }
    input_.clear();
    output_.clear();
    streaming_ = false;
  }

  void queue(const std::string & payload)
  {
    if (output_.size() > 262144) {
      RCLCPP_WARN(get_logger(), "Lidar client not reading; closing stale connection");
      disconnect();
      return;
    }
    output_ += cola::frame(payload);
  }

  void reply(
    const std::string & kind, const std::string & name,
    const std::string & data = {})
  {
    queue(kind + " " + name + (data.empty() ? "" : " " + data));
  }

  void command(const std::string & payload)
  {
    const auto split = payload.find(' ', 4);
    const auto kind = payload.substr(0, 3);
    const auto name = payload.substr(4, split == std::string::npos ? split : split - 4);
    const auto args =
      split == std::string::npos ? std::string() : payload.substr(split + 1);
    RCLCPP_DEBUG(get_logger(), "CoLa B %s %s", kind.c_str(), name.c_str());
    if (kind == "sEN" && name == "LMDscandata" && args.size() == 1) {
      streaming_ = args[0] != 0;
      reply("sEA", name, std::string(1, streaming_ ? 1 : 0));
      return;
    }
    if (kind == "sMN") {
      if (name == "SetAccessMode" || name == "Run") {
        reply("sAN", name, std::string(1, 1));
        return;
      }
      if (name == "LMCstartmeas" || name == "LMCstopmeas") {
        measuring_ = name == "LMCstartmeas";
        reply("sAN", name, std::string(1, 0));
        return;
      }
    }
    if (kind == "sWN") {
      if (name == "LMDscandatacfg" && args.size() >= 4) {
        intensity_ = args[2] != 0;
        intensity16_ = args[3] != 0;
        settings_[name] = args;
        reply("sWA", name);
        return;
      }
      if (name == "LMPoutputRange" || name == "EIHstCola" ||
        name == "FREchoFilter")
      {
        if (name == "EIHstCola" && args != std::string(1, 1)) {
          queue(std::string("sFA ") + char(2));
          return;
        }
        reply("sWA", name);
        return;
      }
    }
    if (kind == "sRN") {
      std::string data;
      if (name == "DeviceIdent") {
        data = cola::text("TiM571") + cola::text("V1.0");
      } else if (name == "FirmwareVersion") {
        data = cola::text("V1.0");
      } else if (name == "SerialNumber") {
        data = cola::text("10000001");
      } else if (name == "LocationName") {
        data = cola::text("RoboSoft simulator");
      } else if (name == "SCdevicestate") {
        data = std::string(1, 1);
      } else if (name == "ODoprh" || name == "ODpwrc") {
        cola::number(data, 1, 4);
      } else if (name == "LMPoutputRange") {
        cola::number(data, 1, 2);
        cola::number(data, 3333, 4);
        cola::number(data, static_cast<uint32_t>(-450000), 4);
        cola::number(data, 2250000, 4);
      } else if (name == "LMPscancfg") {
        cola::number(data, 1500, 4);
        cola::number(data, 1, 2);
        cola::number(data, 3333, 4);
        cola::number(data, static_cast<uint32_t>(-450000), 4);
        cola::number(data, 2250000, 4);
      } else if (name == "LMDscandatacfg") {
        data = settings_.count(name) ? settings_[name] :
          std::string("\x01\x00\x01\x01\x00\x00\x00\x00\x00\x00\x01\x00\x01", 13);
      } else if (name == "LMDscandata" && cloudFresh()) {
        queue(cola::scan(ranges_, ++counter_, ticks(), intensity_, intensity16_, true));
        return;
      } else {
        queue(std::string("sFA ") + char(2));
        return;
      }
      reply("sRA", name, data);
      return;
    }
    RCLCPP_WARN(
      get_logger(), "Unsupported CoLa command: %s %s", kind.c_str(), name.c_str());
    queue(std::string("sFA ") + char(2));
  }

  void pollSocket()
  {
    if (client_ < 0) {
      client_ = accept4(listener_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
      if (client_ < 0) {
        return;
      }
      RCLCPP_INFO(get_logger(), "Lidar TCP client connected");
    }
    char buffer[8192];
    const auto received = recv(client_, buffer, sizeof(buffer), 0);
    if (received == 0 ||
      (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
    {
      disconnect();
      return;
    }
    if (received > 0) {
      input_.append(buffer, received);
    }
    while (input_.size() >= 8) {
      if (input_.compare(0, 4, std::string(4, '\x02')) != 0) {
        disconnect();
        return;
      }
      const auto length = cola::read(input_, 4, 4);
      if (length > 65536 || length < 5) {
        disconnect();
        return;
      }
      if (input_.size() < length + 9) {
        break;
      }
      const auto payload = input_.substr(8, length);
      if (cola::frame(payload) != input_.substr(0, length + 9)) {
        disconnect();
        return;
      }
      input_.erase(0, length + 9);
      command(payload);
      if (client_ < 0) {
        return;
      }
    }
    if (!output_.empty()) {
      const auto sent = send(client_, output_.data(), output_.size(), MSG_NOSIGNAL);
      if (sent > 0) {
        output_.erase(0, sent);
      } else if (
        sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      {
        disconnect();
      }
    }
  }

  int listener_{-1};
  int client_{-1};
  bool streaming_{false};
  bool measuring_{true};
  bool intensity_{true};
  bool intensity16_{true};
  bool cloud_received_{false};
  double cloud_timeout_{1.0};
  uint16_t counter_{0};
  Clock::time_point boot_{Clock::now()};
  Clock::time_point last_cloud_{boot_};
  std::vector<uint16_t> ranges_;
  std::string input_;
  std::string output_;
  std::map<std::string, std::string> settings_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
};

}  // namespace robosoft_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robosoft_simulator::SickTim5xxSimulator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sick_tim5xx_simulator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

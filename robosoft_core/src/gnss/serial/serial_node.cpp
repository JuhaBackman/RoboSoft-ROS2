// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "robosoft_interfaces/topics.hpp"

using namespace std::chrono_literals;

namespace robosoft_core
{

class SerialNode final : public rclcpp::Node
{
public:
  explicit SerialNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("serial_node", options)
  {
    device_ = declare_parameter<std::string>("device", "");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    max_write_queue_bytes_ = static_cast<std::size_t>(std::max<int64_t>(
      1, declare_parameter<int>("max_write_queue_bytes", 65536)));
    line_pub_ = create_publisher<std_msgs::msg::String>(
      robosoft_interfaces::kSerialLineTopic, rclcpp::QoS(100));
    write_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      robosoft_interfaces::kSerialWriteTopic, rclcpp::SensorDataQoS(),
      std::bind(&SerialNode::onWrite, this, std::placeholders::_1));
    io_timer_ = create_wall_timer(20ms, std::bind(&SerialNode::update, this));
    RCLCPP_INFO(
      get_logger(), "Serial device configured at '%s' (%d baud)",
      device_.c_str(), baud_rate_);
  }

  ~SerialNode() override
  {
    closePort();
  }

private:
  static speed_t baudConstant(int baud_rate)
  {
    switch (baud_rate) {
      case 4800: return B4800;
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      default: return 0;
    }
  }

  bool configureSerialPort()
  {
    if (!::isatty(serial_fd_)) {
      return true;
    }
    const speed_t baud = baudConstant(baud_rate_);
    if (baud == 0) {
      RCLCPP_ERROR(get_logger(), "Unsupported serial baud rate %d", baud_rate_);
      return false;
    }

    termios settings{};
    if (::tcgetattr(serial_fd_, &settings) != 0) {
      RCLCPP_ERROR(
        get_logger(), "Cannot read serial settings: %s", std::strerror(errno));
      return false;
    }
    ::cfmakeraw(&settings);
    ::cfsetispeed(&settings, baud);
    ::cfsetospeed(&settings, baud);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cflag &= ~PARENB;
    settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
    if (::tcsetattr(serial_fd_, TCSANOW, &settings) != 0) {
      RCLCPP_ERROR(
        get_logger(), "Cannot configure serial port: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  void openPort()
  {
    if (device_.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Serial device is not configured");
      return;
    }
    serial_fd_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot open serial device '%s': %s",
        device_.c_str(), std::strerror(errno));
      return;
    }
    if (!configureSerialPort()) {
      closePort();
      return;
    }
    if (!port_open_logged_) {
      RCLCPP_INFO(get_logger(), "Serial device opened");
      port_open_logged_ = true;
    }
  }

  void closePort()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void onWrite(const std_msgs::msg::UInt8MultiArray & message)
  {
    if (message.data.empty()) {
      return;
    }
    if (message.data.size() > max_write_queue_bytes_ ||
      write_buffer_.size() - write_offset_ >
      max_write_queue_bytes_ - message.data.size())
    {
      RCLCPP_ERROR(get_logger(), "Serial write queue overflow; dropping data");
      return;
    }
    if (write_offset_ == write_buffer_.size()) {
      write_buffer_.clear();
      write_offset_ = 0U;
    } else if (write_offset_ > 0U) {
      write_buffer_.erase(
        write_buffer_.begin(),
        write_buffer_.begin() + static_cast<std::ptrdiff_t>(write_offset_));
      write_offset_ = 0U;
    }
    write_buffer_.insert(
      write_buffer_.end(), message.data.begin(), message.data.end());
  }

  void update()
  {
    if (serial_fd_ < 0) {
      openPort();
      return;
    }
    readInput();
    if (serial_fd_ >= 0) {
      writeOutput();
    }
  }

  void readInput()
  {
    char data[1024];
    for (;;) {
      const ssize_t count = ::read(serial_fd_, data, sizeof(data));
      if (count > 0) {
        input_buffer_.append(data, static_cast<std::size_t>(count));
        publishLines();
      } else if (count == 0) {
        closePort();
        return;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return;
      } else {
        RCLCPP_ERROR(
          get_logger(), "Serial input read failed: %s", std::strerror(errno));
        closePort();
        return;
      }
    }
  }

  void writeOutput()
  {
    while (write_offset_ < write_buffer_.size()) {
      const ssize_t count = ::write(
        serial_fd_, write_buffer_.data() + write_offset_,
        write_buffer_.size() - write_offset_);
      if (count > 0) {
        write_offset_ += static_cast<std::size_t>(count);
      } else if (count < 0 &&
        (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
      {
        return;
      } else {
        RCLCPP_ERROR(
          get_logger(), "Serial output write failed: %s", std::strerror(errno));
        closePort();
        return;
      }
    }
    write_buffer_.clear();
    write_offset_ = 0U;
  }

  void publishLines()
  {
    for (std::size_t newline; (newline = input_buffer_.find('\n')) !=
      std::string::npos; )
    {
      std_msgs::msg::String output;
      output.data = input_buffer_.substr(0, newline);
      input_buffer_.erase(0, newline + 1);
      if (!output.data.empty() && output.data.back() == '\r') {
        output.data.pop_back();
      }
      if (!output.data.empty()) {
        line_pub_->publish(output);
      }
    }
    if (input_buffer_.size() > 8192) {
      input_buffer_.clear();
      RCLCPP_WARN(get_logger(), "Discarded oversized serial input line");
    }
  }

  std::string device_;
  std::string input_buffer_;
  std::vector<uint8_t> write_buffer_;
  std::size_t write_offset_{0U};
  std::size_t max_write_queue_bytes_{65536U};
  int serial_fd_{-1};
  int baud_rate_{115200};
  bool port_open_logged_{false};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr line_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr write_sub_;
  rclcpp::TimerBase::SharedPtr io_timer_;
};

}  // namespace robosoft_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_core::SerialNode>());
  rclcpp::shutdown();
  return 0;
}

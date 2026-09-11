// Copyright 2026 Juha Backman / Natural Resources Institute Finland
// SPDX-License-Identifier: GPL-3.0-only

#include "robosoft_simulator/nmea0183.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <ctime>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace robosoft_simulator
{

/** Converts simulated vehicle odometry into NMEA 0183 on a serial endpoint. */
class GnssSerialSimulatorNode final : public rclcpp::Node
{
public:
  GnssSerialSimulatorNode()
  : Node("gnss_serial_simulator_node")
  {
    device_ = declare_parameter<std::string>("device", "/tmp/aki_gnss_sim");
    reference_latitude_ = declare_parameter<double>("reference_latitude", 60.194925);
    reference_longitude_ = declare_parameter<double>("reference_longitude", 25.130877);
    altitude_m_ = declare_parameter<double>("altitude_m", 20.0);
    horizontal_stddev_m_ = std::max(
      0.001, declare_parameter<double>("horizontal_stddev_m", 0.02));
    vertical_stddev_m_ = std::max(
      0.001, declare_parameter<double>("vertical_stddev_m", 0.04));
    fix_quality_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<int>("fix_quality", 4), 0, 8));
    satellites_ = static_cast<int>(std::clamp<std::int64_t>(
      declare_parameter<int>("satellites", 18), 0, 99));
    publish_rate_hz_ = std::max(1.0, declare_parameter<double>("publish_rate_hz", 10.0));
    baud_rate_ = declare_parameter<int>("baud_rate", 230400);
    measurement_delay_s_ = std::max(
      0.0, declare_parameter<double>("measurement_delay_s", 0.1));

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "vehicle/odometry", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        if (message) {
          odometry_samples_.push_back({now(), *message});
          if (odometry_samples_.size() > 100U) odometry_samples_.pop_front();
        }
      });
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      [this]() {publish();});
    RCLCPP_INFO(
      get_logger(), "GNSS serial simulator ready: %s at %.1f Hz",
      device_.c_str(), publish_rate_hz_);
  }

  ~GnssSerialSimulatorNode() override
  {
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  bool openDevice()
  {
    if (serial_fd_ >= 0) {
      struct stat path_status {};
      struct stat descriptor_status {};
      if (stat(device_.c_str(), &path_status) == 0 &&
        fstat(serial_fd_, &descriptor_status) == 0 &&
        path_status.st_dev == descriptor_status.st_dev &&
        path_status.st_ino == descriptor_status.st_ino)
      {
        return true;
      }
      RCLCPP_INFO(
        get_logger(), "GNSS serial endpoint changed; reopening %s",
        device_.c_str());
      close(serial_fd_);
      serial_fd_ = -1;
    }
    serial_fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot open GNSS serial endpoint '%s': %s",
        device_.c_str(), std::strerror(errno));
      return false;
    }
    termios settings{};
    if (tcgetattr(serial_fd_, &settings) != 0) {
      RCLCPP_ERROR(
        get_logger(), "Cannot read GNSS serial settings for '%s': %s",
        device_.c_str(), std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    const speed_t baud = baudConstant(baud_rate_);
    if (baud == 0) {
      RCLCPP_ERROR(get_logger(), "Unsupported GNSS serial baud rate: %d", baud_rate_);
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    cfmakeraw(&settings);
    settings.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    cfsetispeed(&settings, baud);
    cfsetospeed(&settings, baud);
    if (tcsetattr(serial_fd_, TCSANOW, &settings) != 0) {
      RCLCPP_ERROR(
        get_logger(), "Cannot configure GNSS serial endpoint '%s': %s",
        device_.c_str(), std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }
    RCLCPP_INFO(get_logger(), "GNSS serial endpoint opened: %s", device_.c_str());
    return true;
  }

  static speed_t baudConstant(int baud_rate)
  {
    switch (baud_rate) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      default: return 0;
    }
  }

  static std::string utcTime()
  {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    gmtime_r(&now, &utc);
    std::ostringstream output;
    output << std::setfill('0') << std::setw(2) << utc.tm_hour <<
      std::setw(2) << utc.tm_min << std::setw(2) << utc.tm_sec << ".00";
    return output.str();
  }

  void publish()
  {
    const auto cutoff = now() - rclcpp::Duration::from_seconds(measurement_delay_s_);
    while (odometry_samples_.size() > 1U &&
      odometry_samples_[1].received <= cutoff)
    {
      odometry_samples_.pop_front();
    }
    if (odometry_samples_.empty() || odometry_samples_.front().received > cutoff ||
      !openDevice())
    {
      return;
    }
    const auto & odometry = odometry_samples_.front().odometry;
    // Consume optional RTCM bytes written by the robot-side serial node. The
    // current kinematic GNSS model does not alter its fix from corrections.
    char corrections[1024];
    while (read(serial_fd_, corrections, sizeof(corrections)) > 0) {}
    const double latitude = nmea0183::latitudeFromNorthing(
      reference_latitude_, odometry.pose.pose.position.y);
    const double longitude = nmea0183::longitudeFromEasting(
      reference_latitude_, reference_longitude_, odometry.pose.pose.position.x);
    const auto & orientation = odometry.pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    const double track_degrees = nmea0183::wrapDegrees(
      90.0 - yaw * 180.0 / nmea0183::kPi);
    const double speed_m_s = std::hypot(
      odometry.twist.twist.linear.x, odometry.twist.twist.linear.y);
    const std::string time = utcTime();

    std::ostringstream gga;
    gga << "GPGGA," << time << ',' << nmea0183::coordinate(latitude, true) << ',' <<
      (latitude < 0.0 ? 'S' : 'N') << ',' << nmea0183::coordinate(longitude, false) << ',' <<
      (longitude < 0.0 ? 'W' : 'E') << ',' << fix_quality_ << ',' << satellites_ <<
      ",0.6," << std::fixed << std::setprecision(3) << altitude_m_ << ",M,0.0,M,,";
    std::ostringstream gst;
    gst << "GPGST," << time << ',' << horizontal_stddev_m_ << ",0,0,0," <<
      horizontal_stddev_m_ << ',' << horizontal_stddev_m_ << ',' << vertical_stddev_m_;
    std::ostringstream vtg;
    vtg << "GPVTG," << std::fixed << std::setprecision(4) << track_degrees <<
      ",T,,M,,N," << speed_m_s * 3.6 << ",K";
    std::ostringstream pkhm;
    pkhm << "PKHM,0,0,0,0," << std::fixed << std::setprecision(4) <<
      track_degrees << ",0.0,0.0";
    writeAll(nmea0183::sentence(gga.str()) + nmea0183::sentence(gst.str()) +
      nmea0183::sentence(vtg.str()) + nmea0183::sentence(pkhm.str()));
  }

  void writeAll(const std::string & data)
  {
    pending_output_.append(data);
    std::size_t offset = 0U;
    while (offset < pending_output_.size()) {
      const ssize_t count = write(
        serial_fd_, pending_output_.data() + offset, pending_output_.size() - offset);
      if (count > 0) {
        offset += static_cast<std::size_t>(count);
      } else if (count < 0 && errno == EINTR) {
        continue;
      } else if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // Keep the unwritten suffix for the next timer cycle. A nonblocking
        // serial endpoint can transiently reject writes while its USB/UART
        // transmit buffer is being drained.
        break;
      } else {
        RCLCPP_WARN(get_logger(), "GNSS serial write failed: %s", std::strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        pending_output_.clear();
        return;
      }
    }
    pending_output_.erase(0U, offset);
    if (pending_output_.size() > 65536U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GNSS serial output is blocked; discarding %zu bytes of stale simulated data",
        pending_output_.size());
      pending_output_.clear();
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  struct OdometrySample
  {
    rclcpp::Time received;
    nav_msgs::msg::Odometry odometry;
  };
  std::deque<OdometrySample> odometry_samples_;
  std::string device_;
  int serial_fd_{-1};
  int baud_rate_{230400};
  std::string pending_output_;
  int fix_quality_{4};
  int satellites_{18};
  double reference_latitude_{60.194925};
  double reference_longitude_{25.130877};
  double altitude_m_{20.0};
  double horizontal_stddev_m_{0.02};
  double vertical_stddev_m_{0.04};
  double publish_rate_hz_{10.0};
  double measurement_delay_s_{0.1};
};

}  // namespace robosoft_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robosoft_simulator::GnssSerialSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}

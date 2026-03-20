// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>


constexpr const char *DEFAULT_IMU_PROTOCOL = "auto";
constexpr const char *DEFAULT_IMU_DEVICE = "auto";
constexpr const char *DEFAULT_IMU_BAUDRATE = "auto";
constexpr double DEFAULT_IMU_TIMEOUT = 0.01;
constexpr double DEFAULT_IMU_PROBE_DURATION = 0.5;

constexpr uint8_t PACKET_SYNC_1 = 0x75;
constexpr uint8_t PACKET_SYNC_2 = 0x65;
constexpr uint8_t HIWONDER_SYNC = 0x55;
constexpr size_t HIWONDER_FRAME_LENGTH = 11;


enum class IMUProtocol {
  AUTO,
  HIWONDER,
  PACKET,
};


struct IMUCliOptions {
  std::string protocol = DEFAULT_IMU_PROTOCOL;
  std::string device = DEFAULT_IMU_DEVICE;
  std::string baudrate = DEFAULT_IMU_BAUDRATE;
  double timeout = DEFAULT_IMU_TIMEOUT;
  double probe_duration = DEFAULT_IMU_PROBE_DURATION;
};


struct IMUConfiguration {
  IMUProtocol protocol = IMUProtocol::HIWONDER;
  std::string device;
  speed_t baudrate = B460800;
  int baudrate_value = 460800;
  double timeout = DEFAULT_IMU_TIMEOUT;
};


bool parse_imu_cli_options(
    int argc,
    char **argv,
    IMUCliOptions *options,
    bool *show_help,
    std::string *error_message);
void print_imu_usage(const char *program_name);
IMUConfiguration resolve_imu_configuration(const IMUCliOptions &options);
const char *imu_protocol_name(IMUProtocol protocol);


class IMU {
  public:
    float quaternion[4];        // quaternion, (w, x, y, z)
    float angular_velocity[3];  // angular velocity, (x, y, z), rad/s
    float angle[3];             // Euler angle, (roll, pitch, yaw), deg
    float gravity_vector[3];    // gravity vector, (x, y, z), m/s^2

    IMU(const IMUConfiguration &configuration);

    ~IMU();

    ssize_t init();

    bool update_reading();
    bool probe(double duration_seconds);

    const std::string &device() const { return path_; }
    int baudrate_value() const { return baudrate_value_; }
    IMUProtocol protocol() const { return protocol_; }

  private:
    int fd = -1;
    std::string path_;
    speed_t baudrate_;
    int baudrate_value_;
    IMUProtocol protocol_;
    double timeout_;

    bool read_packet_frame();
    bool read_hiwonder_frame();
};

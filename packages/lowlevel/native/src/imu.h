// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

#include <array>
#include <fcntl.h>
#include <glob.h>
#include <math.h>
#include <mutex>
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


struct IMUSnapshot {
  double captured_at = 0.0;
  double last_frame_timestamp = 0.0;
  double last_quaternion_timestamp = 0.0;
  double last_angular_velocity_timestamp = 0.0;
  std::array<float, 4> quaternion = {0.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 3> angular_velocity = {0.0f, 0.0f, 0.0f};
  std::array<float, 3> angle = {0.0f, 0.0f, 0.0f};
  std::array<float, 3> gravity_vector = {0.0f, 0.0f, 0.0f};
  bool quaternion_ready = false;
  bool angular_velocity_ready = false;

  double quaternion_age_seconds() const;
  double angular_velocity_age_seconds() const;
  bool is_ready(double max_staleness_seconds = -1.0) const;
};


class IMU {
  public:
    float quaternion[4];        // quaternion, (w, x, y, z)
    float angular_velocity[3];  // angular velocity, (x, y, z), rad/s
    float angle[3];             // Euler angle, (roll, pitch, yaw), deg
    float gravity_vector[3];    // gravity vector, (x, y, z), m/s^2

    IMU(const IMUConfiguration &configuration);

    ~IMU();

    ssize_t init();

    int read_frame_type();
    bool update_reading();
    bool probe(double duration_seconds);
    IMUSnapshot snapshot() const;
    bool is_ready(double max_staleness_seconds = -1.0) const;
    bool wait_until_ready(
        double timeout_seconds,
        double max_staleness_seconds = -1.0,
        double poll_interval_seconds = 0.01);
    void adopt_file_descriptor_for_testing(int file_descriptor);

    const std::string &device() const { return path_; }
    int baudrate_value() const { return baudrate_value_; }
    IMUProtocol protocol() const { return protocol_; }

  private:
    int fd = -1;
    bool owns_fd_ = true;
    std::string path_;
    speed_t baudrate_;
    int baudrate_value_;
    IMUProtocol protocol_;
    double timeout_;
    mutable std::mutex state_mutex_;
    bool quaternion_ready_ = false;
    bool angular_velocity_ready_ = false;
    double last_frame_timestamp_ = 0.0;
    double last_quaternion_timestamp_ = 0.0;
    double last_angular_velocity_timestamp_ = 0.0;
    std::vector<uint8_t> hiwonder_read_buffer_;

    bool read_packet_frame();
    bool read_hiwonder_frame();
    int read_hiwonder_frame_type();
    int try_extract_hiwonder_frame_type_from_buffer();
    void reset_state();
    void apply_packet_sample(const float *uart_buffer, double frame_timestamp);
    void apply_hiwonder_frame(
        uint8_t frame_type,
        int16_t data1,
        int16_t data2,
        int16_t data3,
        int16_t data4,
        double frame_timestamp);
    ssize_t read_some(uint8_t *buffer, size_t buffer_size, double timeout_seconds);
    bool read_exact(uint8_t *buffer, size_t buffer_size, double timeout_seconds);
    static double now_seconds();
    static uint8_t compute_hiwonder_checksum(const uint8_t *frame);
    static bool is_supported_hiwonder_frame_type(uint8_t frame_type);
    static bool is_probe_hiwonder_frame_type(int frame_type);
};

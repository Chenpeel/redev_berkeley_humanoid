// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <exception>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "imu.h"

namespace {

constexpr uint8_t kHiwonderFrameTypeTime = 0x50;
constexpr uint8_t kHiwonderFrameTypeAcceleration = 0x51;
constexpr uint8_t kHiwonderFrameTypeAngularVelocity = 0x52;
constexpr uint8_t kHiwonderFrameTypeQuaternion = 0x59;

constexpr float kPi = 3.14159265358979323846f;

struct PipePair {
  int read_fd = -1;
  int write_fd = -1;
};

IMUConfiguration make_hiwonder_configuration() {
  IMUConfiguration configuration;
  configuration.protocol = IMUProtocol::HIWONDER;
  configuration.device = "test";
  configuration.baudrate = B9600;
  configuration.baudrate_value = 9600;
  configuration.timeout = 0.01;
  return configuration;
}

PipePair create_pipe_pair() {
  int file_descriptors[2];
  if (pipe(file_descriptors) != 0) {
    throw std::runtime_error(std::string("pipe failed: ") + strerror(errno));
  }
  return PipePair{file_descriptors[0], file_descriptors[1]};
}

void close_if_open(int *file_descriptor) {
  if (*file_descriptor >= 0) {
    close(*file_descriptor);
    *file_descriptor = -1;
  }
}

void write_all(int file_descriptor, const std::vector<uint8_t> &bytes) {
  size_t total_written = 0;
  while (total_written < bytes.size()) {
    const ssize_t bytes_written = write(
        file_descriptor,
        bytes.data() + total_written,
        bytes.size() - total_written);
    if (bytes_written < 0) {
      throw std::runtime_error(std::string("write failed: ") + strerror(errno));
    }
    total_written += static_cast<size_t>(bytes_written);
  }
}

std::array<uint8_t, HIWONDER_FRAME_LENGTH> make_hiwonder_frame(
    uint8_t frame_type,
    int16_t data1,
    int16_t data2,
    int16_t data3,
    int16_t data4) {
  std::array<uint8_t, HIWONDER_FRAME_LENGTH> frame{};
  frame[0] = HIWONDER_SYNC;
  frame[1] = frame_type;
  frame[2] = static_cast<uint8_t>(data1 & 0xFF);
  frame[3] = static_cast<uint8_t>((data1 >> 8) & 0xFF);
  frame[4] = static_cast<uint8_t>(data2 & 0xFF);
  frame[5] = static_cast<uint8_t>((data2 >> 8) & 0xFF);
  frame[6] = static_cast<uint8_t>(data3 & 0xFF);
  frame[7] = static_cast<uint8_t>((data3 >> 8) & 0xFF);
  frame[8] = static_cast<uint8_t>(data4 & 0xFF);
  frame[9] = static_cast<uint8_t>((data4 >> 8) & 0xFF);

  uint16_t checksum = 0;
  for (size_t index = 0; index < HIWONDER_FRAME_LENGTH - 1; ++index) {
    checksum += frame[index];
  }
  frame[10] = static_cast<uint8_t>(checksum & 0xFF);
  return frame;
}

std::vector<uint8_t> concat_bytes(
    std::initializer_list<std::vector<uint8_t>> chunks) {
  std::vector<uint8_t> output;
  for (const std::vector<uint8_t> &chunk : chunks) {
    output.insert(output.end(), chunk.begin(), chunk.end());
  }
  return output;
}

std::vector<uint8_t> to_vector(const std::array<uint8_t, HIWONDER_FRAME_LENGTH> &frame) {
  return std::vector<uint8_t>(frame.begin(), frame.end());
}

bool nearly_equal(float left, float right, float tolerance = 1e-4f) {
  return std::fabs(left - right) <= tolerance;
}

bool expect_true(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "[FAIL] %s\n", message);
    return false;
  }
  return true;
}

bool test_hiwonder_checksum_and_resync() {
  PipePair pipe_pair = create_pipe_pair();
  auto invalid_frame = make_hiwonder_frame(kHiwonderFrameTypeAcceleration, 100, 200, 300, 0);
  invalid_frame[10] ^= 0xFF;
  const auto valid_gyro_frame =
      make_hiwonder_frame(kHiwonderFrameTypeAngularVelocity, 16384, 0, -16384, 0);

  write_all(
      pipe_pair.write_fd,
      concat_bytes(
          {
              std::vector<uint8_t>{0x00, 0x12, 0x34},
              to_vector(invalid_frame),
              to_vector(valid_gyro_frame),
          }));
  close_if_open(&pipe_pair.write_fd);

  IMU imu(make_hiwonder_configuration());
  imu.adopt_file_descriptor_for_testing(pipe_pair.read_fd);
  pipe_pair.read_fd = -1;

  const int frame_type = imu.read_frame_type();
  const IMUSnapshot imu_snapshot = imu.snapshot();

  return expect_true(frame_type == kHiwonderFrameTypeAngularVelocity, "checksum/resync should land on gyro frame") &&
      expect_true(imu_snapshot.angular_velocity_ready, "gyro frame should mark angular velocity ready") &&
      expect_true(!imu_snapshot.quaternion_ready, "gyro frame should not mark quaternion ready") &&
      expect_true(
          nearly_equal(imu_snapshot.angular_velocity[0], 1000.0f * kPi / 180.0f),
          "gyro X scaling should match native rad/s conversion") &&
      expect_true(
          nearly_equal(imu_snapshot.angular_velocity[2], -1000.0f * kPi / 180.0f),
          "gyro Z scaling should match native rad/s conversion");
}

bool test_probe_ignores_time_frame() {
  PipePair pipe_pair = create_pipe_pair();
  const auto time_frame = make_hiwonder_frame(kHiwonderFrameTypeTime, 1, 2, 3, 4);
  write_all(pipe_pair.write_fd, to_vector(time_frame));
  close_if_open(&pipe_pair.write_fd);

  IMU imu(make_hiwonder_configuration());
  imu.adopt_file_descriptor_for_testing(pipe_pair.read_fd);
  pipe_pair.read_fd = -1;

  return expect_true(!imu.probe(0.05), "probe should ignore TIME frame");
}

bool test_probe_accepts_sensor_frame() {
  PipePair pipe_pair = create_pipe_pair();
  const auto acceleration_frame = make_hiwonder_frame(kHiwonderFrameTypeAcceleration, 1, 2, 3, 4);
  write_all(pipe_pair.write_fd, to_vector(acceleration_frame));
  close_if_open(&pipe_pair.write_fd);

  IMU imu(make_hiwonder_configuration());
  imu.adopt_file_descriptor_for_testing(pipe_pair.read_fd);
  pipe_pair.read_fd = -1;

  return expect_true(imu.probe(0.05), "probe should accept acceleration frame");
}

bool test_ready_and_staleness() {
  PipePair pipe_pair = create_pipe_pair();
  const auto gyro_frame = make_hiwonder_frame(kHiwonderFrameTypeAngularVelocity, 100, 0, 0, 0);
  const auto quaternion_frame = make_hiwonder_frame(kHiwonderFrameTypeQuaternion, 32768 / 2, 0, 0, 0);
  write_all(
      pipe_pair.write_fd,
      concat_bytes({to_vector(gyro_frame), to_vector(quaternion_frame)}));
  close_if_open(&pipe_pair.write_fd);

  IMU imu(make_hiwonder_configuration());
  imu.adopt_file_descriptor_for_testing(pipe_pair.read_fd);
  pipe_pair.read_fd = -1;

  if (!expect_true(imu.read_frame_type() == kHiwonderFrameTypeAngularVelocity, "first frame should be gyro")) {
    return false;
  }
  if (!expect_true(imu.read_frame_type() == kHiwonderFrameTypeQuaternion, "second frame should be quaternion")) {
    return false;
  }
  if (!expect_true(imu.wait_until_ready(0.01, 0.05), "imu should become ready after gyro + quaternion")) {
    return false;
  }
  if (!expect_true(imu.is_ready(0.05), "imu should initially be fresh")) {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  return expect_true(!imu.is_ready(0.01), "imu should become stale when max staleness is exceeded");
}

}  // namespace

int main() {
  struct TestCase {
    const char *name;
    bool (*run)();
  };

  const std::array<TestCase, 4> test_cases = {{
      {"checksum_and_resync", test_hiwonder_checksum_and_resync},
      {"probe_ignores_time", test_probe_ignores_time_frame},
      {"probe_accepts_sensor", test_probe_accepts_sensor_frame},
      {"ready_and_staleness", test_ready_and_staleness},
  }};

  size_t passed = 0;
  for (const TestCase &test_case : test_cases) {
    try {
      if (test_case.run()) {
        std::printf("[PASS] %s\n", test_case.name);
        ++passed;
      } else {
        std::fprintf(stderr, "[FAIL] %s\n", test_case.name);
      }
    } catch (const std::exception &error) {
      std::fprintf(stderr, "[FAIL] %s: %s\n", test_case.name, error.what());
    }
  }

  if (passed != test_cases.size()) {
    std::fprintf(stderr, "%zu/%zu tests passed\n", passed, test_cases.size());
    return 1;
  }

  std::printf("%zu/%zu tests passed\n", passed, test_cases.size());
  return 0;
}

// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include "imu.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <limits>
#include <limits.h>
#include <poll.h>
#include <stdexcept>
#include <thread>


namespace {

constexpr std::array<const char *, 3> kPreferredDevices = {
    "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.1:1.0",
    "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0",
};

constexpr std::array<const char *, 4> kDevicePatterns = {
    "/dev/serial/by-id/*",
    "/dev/serial/by-path/*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
};

constexpr std::array<int, 7> kHiwonderDefaultBaudrates = {
    460800, 115200, 230400, 9600, 19200, 38400, 57600,
};

constexpr std::array<int, 1> kPacketDefaultBaudrates = {
    1000000,
};

constexpr uint8_t kHiwonderFrameTypeTime = 0x50;
constexpr uint8_t kHiwonderFrameTypeAcceleration = 0x51;
constexpr uint8_t kHiwonderFrameTypeAngularVelocity = 0x52;
constexpr uint8_t kHiwonderFrameTypeAngle = 0x53;
constexpr uint8_t kHiwonderFrameTypeMagneticField = 0x54;
constexpr uint8_t kHiwonderFrameTypeQuaternion = 0x59;
constexpr int kPacketFrameType = 0x100;

int timeout_seconds_to_poll_ms(double timeout_seconds) {
  if (timeout_seconds <= 0.0) {
    return 0;
  }
  return static_cast<int>(std::ceil(timeout_seconds * 1000.0));
}

int16_t read_int16_le(const uint8_t *buffer) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(buffer[0]) |
      (static_cast<uint16_t>(buffer[1]) << 8));
}

float deg2rad_local(float deg) {
  return deg * static_cast<float>(M_PI) / 180.0f;
}

bool string_equals(const std::string &left, const char *right) {
  return left == right;
}

IMUProtocol parse_protocol(const std::string &protocol) {
  if (string_equals(protocol, "auto")) {
    return IMUProtocol::AUTO;
  }
  if (string_equals(protocol, "hiwonder")) {
    return IMUProtocol::HIWONDER;
  }
  if (string_equals(protocol, "packet")) {
    return IMUProtocol::PACKET;
  }
  throw std::runtime_error("Unsupported IMU protocol: " + protocol);
}

speed_t baudrate_to_speed(int baudrate_value) {
  switch (baudrate_value) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 1000000:
      return B1000000;
    default:
      throw std::runtime_error("Unsupported IMU baudrate: " + std::to_string(baudrate_value));
  }
}

std::vector<std::string> discover_imu_devices() {
  std::vector<std::string> devices;
  std::vector<std::string> resolved_paths;

  auto append_candidate = [&](const std::string &candidate) {
    if (access(candidate.c_str(), F_OK) != 0) {
      return;
    }

    char resolved_buffer[PATH_MAX];
    if (realpath(candidate.c_str(), resolved_buffer) == nullptr) {
      return;
    }

    std::string resolved_path(resolved_buffer);
    for (const std::string &existing : resolved_paths) {
      if (existing == resolved_path) {
        return;
      }
    }

    resolved_paths.push_back(resolved_path);
    devices.push_back(candidate);
  };

  for (const char *candidate : kPreferredDevices) {
    append_candidate(candidate);
  }

  for (const char *pattern : kDevicePatterns) {
    glob_t glob_result{};
    if (glob(pattern, 0, nullptr, &glob_result) != 0) {
      globfree(&glob_result);
      continue;
    }

    for (size_t index = 0; index < glob_result.gl_pathc; ++index) {
      append_candidate(glob_result.gl_pathv[index]);
    }
    globfree(&glob_result);
  }

  return devices;
}

std::vector<int> build_baudrate_candidates(IMUProtocol protocol, const std::string &baudrate_argument) {
  if (!string_equals(baudrate_argument, "auto")) {
    return {std::stoi(baudrate_argument)};
  }

  if (protocol == IMUProtocol::PACKET) {
    return std::vector<int>(kPacketDefaultBaudrates.begin(), kPacketDefaultBaudrates.end());
  }

  return std::vector<int>(kHiwonderDefaultBaudrates.begin(), kHiwonderDefaultBaudrates.end());
}

IMUConfiguration make_configuration(
    IMUProtocol protocol,
    const std::string &device,
    int baudrate_value,
    double timeout) {
  IMUConfiguration configuration;
  configuration.protocol = protocol;
  configuration.device = device;
  configuration.baudrate = baudrate_to_speed(baudrate_value);
  configuration.baudrate_value = baudrate_value;
  configuration.timeout = timeout;
  return configuration;
}

std::vector<IMUConfiguration> build_probe_configurations(const IMUCliOptions &options) {
  const IMUProtocol requested_protocol = parse_protocol(options.protocol);
  std::vector<IMUProtocol> protocols;
  if (requested_protocol == IMUProtocol::AUTO) {
    protocols = {IMUProtocol::HIWONDER, IMUProtocol::PACKET};
  } else {
    protocols = {requested_protocol};
  }

  std::vector<std::string> devices;
  if (string_equals(options.device, "auto")) {
    devices = discover_imu_devices();
    if (devices.empty()) {
      throw std::runtime_error("No IMU serial device found. Pass --device to specify the serial port explicitly.");
    }
  } else {
    devices = {options.device};
  }

  std::vector<IMUConfiguration> configurations;
  for (IMUProtocol protocol : protocols) {
    for (const std::string &device : devices) {
      const std::vector<int> baudrates = build_baudrate_candidates(protocol, options.baudrate);
      for (int baudrate_value : baudrates) {
        configurations.push_back(make_configuration(protocol, device, baudrate_value, options.timeout));
      }
    }
  }
  return configurations;
}

}  // namespace


double IMUSnapshot::quaternion_age_seconds() const {
  if (!quaternion_ready || last_quaternion_timestamp <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, captured_at - last_quaternion_timestamp);
}


double IMUSnapshot::angular_velocity_age_seconds() const {
  if (!angular_velocity_ready || last_angular_velocity_timestamp <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::max(0.0, captured_at - last_angular_velocity_timestamp);
}


bool IMUSnapshot::is_ready(double max_staleness_seconds) const {
  if (!quaternion_ready || !angular_velocity_ready) {
    return false;
  }
  if (max_staleness_seconds < 0.0) {
    return true;
  }
  return quaternion_age_seconds() <= max_staleness_seconds &&
      angular_velocity_age_seconds() <= max_staleness_seconds;
}


bool parse_imu_cli_options(
    int argc,
    char **argv,
    IMUCliOptions *options,
    bool *show_help,
    std::string *error_message) {
  *show_help = false;
  error_message->clear();

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto require_value = [&](const char *name) -> const char * {
      if (index + 1 >= argc) {
        *error_message = std::string("Missing value for ") + name;
        return nullptr;
      }
      ++index;
      return argv[index];
    };

    if (argument == "--help" || argument == "-h") {
      *show_help = true;
      return true;
    }
    if (argument == "--protocol") {
      const char *value = require_value("--protocol");
      if (value == nullptr) {
        return false;
      }
      options->protocol = value;
      continue;
    }
    if (argument == "--device") {
      const char *value = require_value("--device");
      if (value == nullptr) {
        return false;
      }
      options->device = value;
      continue;
    }
    if (argument == "--baudrate") {
      const char *value = require_value("--baudrate");
      if (value == nullptr) {
        return false;
      }
      options->baudrate = value;
      continue;
    }
    if (argument == "--timeout") {
      const char *value = require_value("--timeout");
      if (value == nullptr) {
        return false;
      }
      options->timeout = std::stod(value);
      continue;
    }
    if (argument == "--probe-duration") {
      const char *value = require_value("--probe-duration");
      if (value == nullptr) {
        return false;
      }
      options->probe_duration = std::stod(value);
      continue;
    }

    *error_message = "Unknown argument: " + argument;
    return false;
  }

  return true;
}


void print_imu_usage(const char *program_name) {
  printf(
      "Usage: %s [--protocol auto|hiwonder|packet] [--device PATH|auto] "
      "[--baudrate VALUE|auto] [--timeout SECONDS] [--probe-duration SECONDS]\n",
      program_name);
}


const char *imu_protocol_name(IMUProtocol protocol) {
  switch (protocol) {
    case IMUProtocol::AUTO:
      return "auto";
    case IMUProtocol::HIWONDER:
      return "hiwonder";
    case IMUProtocol::PACKET:
      return "packet";
  }
  return "unknown";
}


IMUConfiguration resolve_imu_configuration(const IMUCliOptions &options) {
  const IMUProtocol requested_protocol = parse_protocol(options.protocol);
  if (
      requested_protocol != IMUProtocol::AUTO &&
      !string_equals(options.device, "auto") &&
      !string_equals(options.baudrate, "auto")) {
    return make_configuration(
        requested_protocol,
        options.device,
        std::stoi(options.baudrate),
        options.timeout);
  }

  std::runtime_error last_error("Unable to detect a working IMU stream configuration.");
  const std::vector<IMUConfiguration> configurations = build_probe_configurations(options);
  for (const IMUConfiguration &configuration : configurations) {
    try {
      IMU imu(configuration);
      if (imu.init() < 0) {
        continue;
      }
      if (imu.probe(options.probe_duration)) {
        return configuration;
      }
    } catch (const std::exception &error) {
      last_error = std::runtime_error(error.what());
    }
  }

  throw std::runtime_error(
      std::string("Unable to detect a working IMU stream configuration. ") +
      last_error.what());
}


IMU::IMU(const IMUConfiguration &configuration) {
  path_ = configuration.device;
  baudrate_ = configuration.baudrate;
  baudrate_value_ = configuration.baudrate_value;
  protocol_ = configuration.protocol;
  timeout_ = configuration.timeout;
  reset_state();
}


IMU::~IMU() {
  if (owns_fd_ && fd >= 0) {
    close(fd);
  }
}


ssize_t IMU::init() {
  if (owns_fd_ && fd >= 0) {
    close(fd);
  }
  owns_fd_ = true;
  fd = open(path_.c_str(), O_RDWR | O_NOCTTY);

  if (fd < 0) {
    printf("[ERROR] <Serial>: Error %i from open: %s\n", errno, strerror(errno));
    return -1;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd, &tty) != 0) {
    printf("[ERROR] <Serial>: Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return -1;
  }

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  // Use poll() for timeout control so 0.01s really means about 10ms, not 100ms.
  tty.c_cc[VTIME] = 0;
  tty.c_cc[VMIN] = 0;

  if (cfsetispeed(&tty, baudrate_) != 0 || cfsetospeed(&tty, baudrate_) != 0) {
    printf("[ERROR] <Serial>: Error %i from cfset[speed]: %s\n", errno, strerror(errno));
    return -1;
  }

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    printf("[ERROR] <Serial>: Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return -1;
  }

  tcflush(fd, TCIFLUSH);

  printf(
      "[INFO] <IMU>: Serial port %s initialized with protocol %s at %d baud\n",
      path_.c_str(),
      imu_protocol_name(protocol_),
      baudrate_value_);
  reset_state();

  usleep(100000);
  return 0;
}


void IMU::reset_state() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::fill(std::begin(quaternion), std::end(quaternion), 0.0f);
  std::fill(std::begin(angular_velocity), std::end(angular_velocity), 0.0f);
  std::fill(std::begin(angle), std::end(angle), 0.0f);
  std::fill(std::begin(gravity_vector), std::end(gravity_vector), 0.0f);
  quaternion_ready_ = false;
  angular_velocity_ready_ = false;
  last_frame_timestamp_ = 0.0;
  last_quaternion_timestamp_ = 0.0;
  last_angular_velocity_timestamp_ = 0.0;
  hiwonder_read_buffer_.clear();
}


double IMU::now_seconds() {
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}


void IMU::adopt_file_descriptor_for_testing(int file_descriptor) {
  if (file_descriptor < 0) {
    throw std::invalid_argument("file_descriptor must be non-negative");
  }
  if (owns_fd_ && fd >= 0) {
    close(fd);
  }
  fd = file_descriptor;
  owns_fd_ = true;
  reset_state();
}


IMUSnapshot IMU::snapshot() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  IMUSnapshot imu_snapshot;
  imu_snapshot.captured_at = now_seconds();
  imu_snapshot.last_frame_timestamp = last_frame_timestamp_;
  imu_snapshot.last_quaternion_timestamp = last_quaternion_timestamp_;
  imu_snapshot.last_angular_velocity_timestamp = last_angular_velocity_timestamp_;
  std::copy(std::begin(quaternion), std::end(quaternion), imu_snapshot.quaternion.begin());
  std::copy(
      std::begin(angular_velocity),
      std::end(angular_velocity),
      imu_snapshot.angular_velocity.begin());
  std::copy(std::begin(angle), std::end(angle), imu_snapshot.angle.begin());
  std::copy(
      std::begin(gravity_vector),
      std::end(gravity_vector),
      imu_snapshot.gravity_vector.begin());
  imu_snapshot.quaternion_ready = quaternion_ready_;
  imu_snapshot.angular_velocity_ready = angular_velocity_ready_;
  return imu_snapshot;
}


bool IMU::is_ready(double max_staleness_seconds) const {
  return snapshot().is_ready(max_staleness_seconds);
}


bool IMU::wait_until_ready(
    double timeout_seconds,
    double max_staleness_seconds,
    double poll_interval_seconds) {
  if (timeout_seconds < 0.0) {
    throw std::invalid_argument("timeout_seconds must be non-negative");
  }
  if (poll_interval_seconds < 0.0) {
    throw std::invalid_argument("poll_interval_seconds must be non-negative");
  }

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_seconds);
  while (true) {
    if (is_ready(max_staleness_seconds)) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(poll_interval_seconds));
  }
}


ssize_t IMU::read_some(uint8_t *buffer, size_t buffer_size, double timeout_seconds) {
  if (fd < 0) {
    return -1;
  }

  while (true) {
    pollfd poll_descriptor{};
    poll_descriptor.fd = fd;
    poll_descriptor.events = POLLIN;

    const int poll_result = poll(&poll_descriptor, 1, timeout_seconds_to_poll_ms(timeout_seconds));
    if (poll_result < 0) {
      if (errno == EINTR) {
        continue;
      }
      printf("[ERROR] <Serial>: Error %i from poll: %s\n", errno, strerror(errno));
      return -1;
    }
    if (poll_result == 0) {
      return 0;
    }

    const ssize_t bytes_read = read(fd, buffer, buffer_size);
    if (bytes_read < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0;
      }
      printf("[ERROR] <Serial>: Error %i from read: %s\n", errno, strerror(errno));
      return -1;
    }
    return bytes_read;
  }
}


bool IMU::read_exact(uint8_t *buffer, size_t buffer_size, double timeout_seconds) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(0.0, timeout_seconds));
  size_t total_bytes_read = 0;

  while (total_bytes_read < buffer_size) {
    double remaining_timeout = timeout_seconds;
    if (timeout_seconds >= 0.0) {
      remaining_timeout =
          std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
      if (remaining_timeout <= 0.0) {
        return false;
      }
    }

    const ssize_t bytes_read = read_some(
        buffer + total_bytes_read,
        buffer_size - total_bytes_read,
        remaining_timeout);
    if (bytes_read <= 0) {
      return false;
    }
    total_bytes_read += static_cast<size_t>(bytes_read);
  }
  return true;
}


void IMU::apply_packet_sample(const float *uart_buffer, double frame_timestamp) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  quaternion[0] = uart_buffer[0];
  quaternion[1] = uart_buffer[1];
  quaternion[2] = uart_buffer[2];
  quaternion[3] = uart_buffer[3];

  angular_velocity[0] = uart_buffer[4];
  angular_velocity[1] = uart_buffer[5];
  angular_velocity[2] = uart_buffer[6];

  quaternion_ready_ = true;
  angular_velocity_ready_ = true;
  last_frame_timestamp_ = frame_timestamp;
  last_quaternion_timestamp_ = frame_timestamp;
  last_angular_velocity_timestamp_ = frame_timestamp;
}


void IMU::apply_hiwonder_frame(
    uint8_t frame_type,
    int16_t data1,
    int16_t data2,
    int16_t data3,
    int16_t data4,
    double frame_timestamp) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  last_frame_timestamp_ = frame_timestamp;

  switch (frame_type) {
    case kHiwonderFrameTypeTime:
      return;
    case kHiwonderFrameTypeAcceleration:
      gravity_vector[0] = data1 * 16.0f / 32768.0f;
      gravity_vector[1] = data2 * 16.0f / 32768.0f;
      gravity_vector[2] = data3 * 16.0f / 32768.0f;
      return;
    case kHiwonderFrameTypeAngularVelocity:
      angular_velocity[0] = deg2rad_local(data1 * 2000.0f / 32768.0f);
      angular_velocity[1] = deg2rad_local(data2 * 2000.0f / 32768.0f);
      angular_velocity[2] = deg2rad_local(data3 * 2000.0f / 32768.0f);
      angular_velocity_ready_ = true;
      last_angular_velocity_timestamp_ = frame_timestamp;
      return;
    case kHiwonderFrameTypeAngle:
      angle[0] = data1 * 180.0f / 32768.0f;
      angle[1] = data2 * 180.0f / 32768.0f;
      angle[2] = data3 * 180.0f / 32768.0f;
      return;
    case kHiwonderFrameTypeMagneticField:
      return;
    case kHiwonderFrameTypeQuaternion:
      quaternion[0] = data1 * 1.0f / 32768.0f;
      quaternion[1] = data2 * 1.0f / 32768.0f;
      quaternion[2] = data3 * 1.0f / 32768.0f;
      quaternion[3] = data4 * 1.0f / 32768.0f;
      quaternion_ready_ = true;
      last_quaternion_timestamp_ = frame_timestamp;
      return;
    default:
      return;
  }
}


uint8_t IMU::compute_hiwonder_checksum(const uint8_t *frame) {
  uint16_t checksum = 0;
  for (size_t index = 0; index < HIWONDER_FRAME_LENGTH - 1; ++index) {
    checksum += frame[index];
  }
  return static_cast<uint8_t>(checksum & 0xFF);
}


bool IMU::is_supported_hiwonder_frame_type(uint8_t frame_type) {
  switch (frame_type) {
    case kHiwonderFrameTypeTime:
    case kHiwonderFrameTypeAcceleration:
    case kHiwonderFrameTypeAngularVelocity:
    case kHiwonderFrameTypeAngle:
    case kHiwonderFrameTypeMagneticField:
    case kHiwonderFrameTypeQuaternion:
      return true;
    default:
      return false;
  }
}


bool IMU::is_probe_hiwonder_frame_type(int frame_type) {
  switch (frame_type) {
    case kHiwonderFrameTypeAcceleration:
    case kHiwonderFrameTypeAngularVelocity:
    case kHiwonderFrameTypeAngle:
    case kHiwonderFrameTypeMagneticField:
    case kHiwonderFrameTypeQuaternion:
      return true;
    default:
      return false;
  }
}


int IMU::try_extract_hiwonder_frame_type_from_buffer() {
  while (true) {
    const auto sync_position = std::find(
        hiwonder_read_buffer_.begin(),
        hiwonder_read_buffer_.end(),
        HIWONDER_SYNC);
    if (sync_position == hiwonder_read_buffer_.end()) {
      hiwonder_read_buffer_.clear();
      return -1;
    }
    if (sync_position != hiwonder_read_buffer_.begin()) {
      hiwonder_read_buffer_.erase(hiwonder_read_buffer_.begin(), sync_position);
    }

    if (hiwonder_read_buffer_.size() < HIWONDER_FRAME_LENGTH) {
      return -1;
    }

    const uint8_t *candidate = hiwonder_read_buffer_.data();
    if (compute_hiwonder_checksum(candidate) != candidate[HIWONDER_FRAME_LENGTH - 1]) {
      hiwonder_read_buffer_.erase(hiwonder_read_buffer_.begin());
      continue;
    }

    const uint8_t frame_type = candidate[1];
    if (!is_supported_hiwonder_frame_type(frame_type)) {
      hiwonder_read_buffer_.erase(hiwonder_read_buffer_.begin());
      continue;
    }

    const int16_t data1 = read_int16_le(candidate + 2);
    const int16_t data2 = read_int16_le(candidate + 4);
    const int16_t data3 = read_int16_le(candidate + 6);
    const int16_t data4 = read_int16_le(candidate + 8);
    apply_hiwonder_frame(frame_type, data1, data2, data3, data4, now_seconds());
    hiwonder_read_buffer_.erase(
        hiwonder_read_buffer_.begin(),
        hiwonder_read_buffer_.begin() + HIWONDER_FRAME_LENGTH);
    return frame_type;
  }
}


int IMU::read_hiwonder_frame_type() {
  while (true) {
    const int frame_type = try_extract_hiwonder_frame_type_from_buffer();
    if (frame_type >= 0) {
      return frame_type;
    }

    uint8_t chunk[HIWONDER_FRAME_LENGTH];
    const ssize_t bytes_read = read_some(chunk, sizeof(chunk), timeout_);
    if (bytes_read <= 0) {
      return -1;
    }
    hiwonder_read_buffer_.insert(
        hiwonder_read_buffer_.end(),
        chunk,
        chunk + bytes_read);
  }
}


bool IMU::read_packet_frame() {
  uint8_t sync_byte = 0;
  if (!read_exact(&sync_byte, 1, timeout_) || sync_byte != PACKET_SYNC_1) {
    return false;
  }

  if (!read_exact(&sync_byte, 1, timeout_) || sync_byte != PACKET_SYNC_2) {
    return false;
  }

  uint8_t packet_size_bytes[sizeof(uint16_t)];
  if (!read_exact(packet_size_bytes, sizeof(packet_size_bytes), timeout_)) {
    return false;
  }
  const uint16_t packet_size = static_cast<uint16_t>(
      static_cast<uint16_t>(packet_size_bytes[0]) |
      (static_cast<uint16_t>(packet_size_bytes[1]) << 8));
  if (packet_size != sizeof(float) * 7) {
    return false;
  }

  float uart_buffer[7];
  if (!read_exact(
          reinterpret_cast<uint8_t *>(uart_buffer),
          packet_size,
          timeout_)) {
    return false;
  }

  apply_packet_sample(uart_buffer, now_seconds());
  return true;
}


bool IMU::read_hiwonder_frame() {
  return read_hiwonder_frame_type() >= 0;
}


int IMU::read_frame_type() {
  if (protocol_ == IMUProtocol::PACKET) {
    return read_packet_frame() ? kPacketFrameType : -1;
  }
  return read_hiwonder_frame_type();
}


bool IMU::update_reading() {
  return read_frame_type() >= 0;
}


bool IMU::probe(double duration_seconds) {
  const auto started_at = std::chrono::steady_clock::now();
  while (true) {
    const int frame_type = read_frame_type();
    if (
        (protocol_ == IMUProtocol::PACKET && frame_type == kPacketFrameType) ||
        (protocol_ == IMUProtocol::HIWONDER && is_probe_hiwonder_frame_type(frame_type))) {
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = now - started_at;
    if (elapsed.count() >= duration_seconds) {
      return false;
    }

    usleep(1000);
  }
}

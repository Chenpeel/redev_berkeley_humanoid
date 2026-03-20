// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include "imu.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits.h>
#include <stdexcept>


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
}


IMU::~IMU() {
  if (fd >= 0) {
    close(fd);
  }
}


ssize_t IMU::init() {
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

  const double timeout_deciseconds = std::ceil(timeout_ * 10.0);
  const cc_t serial_timeout =
      timeout_ <= 0.0 ? 0 : static_cast<cc_t>(std::min(255.0, std::max(1.0, timeout_deciseconds)));
  tty.c_cc[VTIME] = serial_timeout;
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

  quaternion[0] = 0.0f;
  quaternion[1] = 0.0f;
  quaternion[2] = 0.0f;
  quaternion[3] = 0.0f;

  angular_velocity[0] = 0.0f;
  angular_velocity[1] = 0.0f;
  angular_velocity[2] = 0.0f;

  angle[0] = 0.0f;
  angle[1] = 0.0f;
  angle[2] = 0.0f;

  gravity_vector[0] = 0.0f;
  gravity_vector[1] = 0.0f;
  gravity_vector[2] = 0.0f;

  usleep(100000);
  return 0;
}


bool IMU::read_packet_frame() {
  uint8_t sync_byte = 0;
  ssize_t bytes_read = read(fd, &sync_byte, 1);
  if (bytes_read < 1 || sync_byte != PACKET_SYNC_1) {
    return false;
  }

  bytes_read = read(fd, &sync_byte, 1);
  if (bytes_read < 1 || sync_byte != PACKET_SYNC_2) {
    return false;
  }

  uint16_t size = 0;
  bytes_read = read(fd, &size, sizeof(size));
  if (bytes_read < static_cast<ssize_t>(sizeof(size)) || size != 28) {
    return false;
  }

  float uart_buffer[7];
  bytes_read = read(fd, uart_buffer, size);
  if (bytes_read < size) {
    return false;
  }

  quaternion[0] = uart_buffer[0];
  quaternion[1] = uart_buffer[1];
  quaternion[2] = uart_buffer[2];
  quaternion[3] = uart_buffer[3];

  angular_velocity[0] = uart_buffer[4];
  angular_velocity[1] = uart_buffer[5];
  angular_velocity[2] = uart_buffer[6];
  return true;
}


bool IMU::read_hiwonder_frame() {
  uint8_t start = 0;
  ssize_t bytes_read = read(fd, &start, 1);
  if (bytes_read < 1 || start != HIWONDER_SYNC) {
    return false;
  }

  uint8_t frame[HIWONDER_FRAME_LENGTH - 1];
  bytes_read = read(fd, frame, sizeof(frame));
  if (bytes_read < static_cast<ssize_t>(sizeof(frame))) {
    return false;
  }

  const uint8_t frame_type = frame[0];
  const int16_t data1 = read_int16_le(frame + 1);
  const int16_t data2 = read_int16_le(frame + 3);
  const int16_t data3 = read_int16_le(frame + 5);
  const int16_t data4 = read_int16_le(frame + 7);

  switch (frame_type) {
    case 0x50:
      return true;
    case 0x51:
      gravity_vector[0] = data1 * 16.0f / 32768.0f;
      gravity_vector[1] = data2 * 16.0f / 32768.0f;
      gravity_vector[2] = data3 * 16.0f / 32768.0f;
      return true;
    case 0x52:
      angular_velocity[0] = deg2rad_local(data1 * 2000.0f / 32768.0f);
      angular_velocity[1] = deg2rad_local(data2 * 2000.0f / 32768.0f);
      angular_velocity[2] = deg2rad_local(data3 * 2000.0f / 32768.0f);
      return true;
    case 0x53:
      angle[0] = data1 * 180.0f / 32768.0f;
      angle[1] = data2 * 180.0f / 32768.0f;
      angle[2] = data3 * 180.0f / 32768.0f;
      return true;
    case 0x59:
      quaternion[0] = data1 * 1.0f / 32768.0f;
      quaternion[1] = data2 * 1.0f / 32768.0f;
      quaternion[2] = data3 * 1.0f / 32768.0f;
      quaternion[3] = data4 * 1.0f / 32768.0f;
      return true;
    default:
      return true;
  }
}


bool IMU::update_reading() {
  if (protocol_ == IMUProtocol::PACKET) {
    return read_packet_frame();
  }
  return read_hiwonder_frame();
}


bool IMU::probe(double duration_seconds) {
  const auto started_at = std::chrono::steady_clock::now();
  while (true) {
    if (update_reading()) {
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

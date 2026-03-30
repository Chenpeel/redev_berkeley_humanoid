// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <signal.h>

#include <memory>
#include <string>
#include <vector>

#include "real_humanoid.h"


namespace {

RealHumanoid *g_humanoid = nullptr;

struct RuntimeCliOptions {
  IMUCliOptions imu_options;
  std::string left_leg_bus = DEFAULT_LEFT_LEG_BUS;
  std::string right_leg_bus = DEFAULT_RIGHT_LEG_BUS;
  LocomotionSpecificationSource specification_source = LocomotionSpecificationSource::LegacyNative;
  std::string hardware_configuration_path = DEFAULT_HARDWARE_CONFIGURATION_PATH;
};

void handle_keyboard_interrupt(int sig) {
  printf("\n<Main> Caught signal %d\n", sig);
  if (g_humanoid != nullptr) {
    g_humanoid->stop();
  }
}

bool parse_runtime_cli_options(
    int argc,
    char **argv,
    RuntimeCliOptions *options,
    bool *show_help,
    std::string *error_message) {
  std::vector<char *> imu_arguments;
  imu_arguments.push_back(argv[0]);

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

    if (argument == "--left-leg-bus") {
      const char *value = require_value("--left-leg-bus");
      if (value == nullptr) {
        return false;
      }
      options->left_leg_bus = value;
      continue;
    }
    if (argument == "--right-leg-bus") {
      const char *value = require_value("--right-leg-bus");
      if (value == nullptr) {
        return false;
      }
      options->right_leg_bus = value;
      continue;
    }
    if (argument == "--spec-source") {
      const char *value = require_value("--spec-source");
      if (value == nullptr) {
        return false;
      }
      const std::string source = value;
      if (source == "legacy") {
        options->specification_source = LocomotionSpecificationSource::LegacyNative;
        continue;
      }
      if (source == "hardware-config") {
        options->specification_source = LocomotionSpecificationSource::HardwareConfiguration;
        continue;
      }
      *error_message =
          "Unsupported --spec-source value: " + source +
          " (expected legacy or hardware-config)";
      return false;
    }
    if (argument == "--hardware-config") {
      const char *value = require_value("--hardware-config");
      if (value == nullptr) {
        return false;
      }
      options->hardware_configuration_path = value;
      continue;
    }

    imu_arguments.push_back(argv[index]);
  }

  return parse_imu_cli_options(
      static_cast<int>(imu_arguments.size()),
      imu_arguments.data(),
      &options->imu_options,
      show_help,
      error_message);
}

}  // namespace


int main(int argc, char **argv) {
  RuntimeCliOptions runtime_options;
  bool show_help = false;
  std::string error_message;
  if (!parse_runtime_cli_options(argc, argv, &runtime_options, &show_help, &error_message)) {
    fprintf(stderr, "%s\n", error_message.c_str());
    print_imu_usage(argv[0]);
    printf(
        "       [--left-leg-bus NAME] [--right-leg-bus NAME]\n"
        "       [--spec-source legacy|hardware-config] [--hardware-config PATH]\n");
    return 1;
  }
  if (show_help) {
    print_imu_usage(argv[0]);
    printf(
        "       [--left-leg-bus NAME] [--right-leg-bus NAME]\n"
        "       [--spec-source legacy|hardware-config] [--hardware-config PATH]\n");
    return 0;
  }

  printf("<Humanoid> Starting...\n");
  signal(SIGINT, handle_keyboard_interrupt);

  try {
    const IMUConfiguration imu_configuration = resolve_imu_configuration(runtime_options.imu_options);
    printf(
        "<Humanoid> IMU config: protocol=%s device=%s baudrate=%d\n",
        imu_protocol_name(imu_configuration.protocol),
        imu_configuration.device.c_str(),
        imu_configuration.baudrate_value);
    printf(
        "<Humanoid> Leg CAN config: left=%s right=%s\n",
        runtime_options.left_leg_bus.c_str(),
        runtime_options.right_leg_bus.c_str());
    printf(
        "<Humanoid> Spec config: source=%s hardware_config=%s\n",
        locomotion_specification_source_name(runtime_options.specification_source),
        runtime_options.hardware_configuration_path.c_str());

    RealHumanoid humanoid(
        imu_configuration,
        runtime_options.left_leg_bus,
        runtime_options.right_leg_bus,
        runtime_options.specification_source,
        runtime_options.hardware_configuration_path);
    g_humanoid = &humanoid;
    humanoid.run();
  } catch (const std::exception &error) {
    fprintf(stderr, "%s\n", error.what());
    return 1;
  }

  return 0;
}

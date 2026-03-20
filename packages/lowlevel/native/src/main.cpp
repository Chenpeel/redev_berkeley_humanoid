// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <signal.h>

#include <memory>
#include <string>

#include "real_humanoid.h"


namespace {

RealHumanoid *g_humanoid = nullptr;

void handle_keyboard_interrupt(int sig) {
  printf("\n<Main> Caught signal %d\n", sig);
  if (g_humanoid != nullptr) {
    g_humanoid->stop();
  }
}

}  // namespace


int main(int argc, char **argv) {
  IMUCliOptions imu_options;
  bool show_help = false;
  std::string error_message;
  if (!parse_imu_cli_options(argc, argv, &imu_options, &show_help, &error_message)) {
    fprintf(stderr, "%s\n", error_message.c_str());
    print_imu_usage(argv[0]);
    return 1;
  }
  if (show_help) {
    print_imu_usage(argv[0]);
    return 0;
  }

  printf("<Humanoid> Starting...\n");
  signal(SIGINT, handle_keyboard_interrupt);

  try {
    const IMUConfiguration imu_configuration = resolve_imu_configuration(imu_options);
    printf(
        "<Humanoid> IMU config: protocol=%s device=%s baudrate=%d\n",
        imu_protocol_name(imu_configuration.protocol),
        imu_configuration.device.c_str(),
        imu_configuration.baudrate_value);

    RealHumanoid humanoid(imu_configuration);
    g_humanoid = &humanoid;
    humanoid.run();
  } catch (const std::exception &error) {
    fprintf(stderr, "%s\n", error.what());
    return 1;
  }

  return 0;
}

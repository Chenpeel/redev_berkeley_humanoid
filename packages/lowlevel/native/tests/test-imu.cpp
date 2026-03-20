// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <pthread.h>
#include <exception>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "imu.h"


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

  try {
    const IMUConfiguration configuration = resolve_imu_configuration(imu_options);
    IMU imu(configuration);
    ssize_t ret = imu.init();
    if (ret < 0) {
      printf("Error initializing IMU: %s\n", strerror(errno));
      return 1;
    }

    const sched_param sched{.sched_priority = 49};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched);

    printf(
        "Streaming native IMU from %s with protocol %s at %d baud\n",
        imu.device().c_str(),
        imu_protocol_name(imu.protocol()),
        imu.baudrate_value());

    while (1) {
      imu.update_reading();

      printf(
          "qpos: %.3f %.3f %.3f %.3f | gyro(rad/s): %.3f %.3f %.3f | angle(deg): %.2f %.2f %.2f\n",
          imu.quaternion[0],
          imu.quaternion[1],
          imu.quaternion[2],
          imu.quaternion[3],
          imu.angular_velocity[0],
          imu.angular_velocity[1],
          imu.angular_velocity[2],
          imu.angle[0],
          imu.angle[1],
          imu.angle[2]);
    }
  } catch (const std::exception &error) {
    fprintf(stderr, "%s\n", error.what());
    return 1;
  }
}

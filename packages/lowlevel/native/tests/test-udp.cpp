// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <cstring>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>

#include "udp.h"
#include "consts.h"

#define N_OBSERVATIONS  50
#define N_ACTIONS       14

#define FREQUENCY 100


float obs[N_OBSERVATIONS];
float acs[N_ACTIONS];


namespace {

const char *mode_name(uint8_t mode)
{
  switch (mode)
  {
    case 0:
      return "keep-last";
    case 1:
      return "idle";
    case 2:
      return "rl-init";
    case 3:
      return "rl-running";
    default:
      return "unknown";
  }
}

}  // namespace


int main() {
  UDP udp;
  initialize_udp(&udp, "0.0.0.0", JOYSTICK_PORT, "127.0.0.1", JOYSTICK_PORT);

  printf("[INFO] Listening for joystick packets on 0.0.0.0:%d\n", JOYSTICK_PORT);
  printf("[INFO] Packet format: <mode:uint8><vx:float><vy:float><vyaw:float>\n");

  size_t packet_count = 0;
  while (true) {
    uint8_t udp_buffer[13];
    ssize_t actual_bytes = recvfrom(udp.sockfd, udp_buffer, sizeof(udp_buffer), MSG_DONTWAIT, NULL, NULL);
    if (actual_bytes < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        usleep(50000);
        continue;
      }
      printf("[ERROR] recvfrom failed: %s\n", strerror(errno));
      return 1;
    }

    packet_count += 1;
    if (actual_bytes != static_cast<ssize_t>(sizeof(udp_buffer)))
    {
      printf("[WARN] packet #%zu has unexpected size %zd\n", packet_count, actual_bytes);
      continue;
    }

    uint8_t mode = udp_buffer[0];
    float velocity_x = 0.0f;
    float velocity_y = 0.0f;
    float velocity_yaw = 0.0f;
    std::memcpy(&velocity_x, udp_buffer + 1, sizeof(float));
    std::memcpy(&velocity_y, udp_buffer + 5, sizeof(float));
    std::memcpy(&velocity_yaw, udp_buffer + 9, sizeof(float));

    printf(
        "[PACKET %zu] mode=%u(%s) vx=%+.3f vy=%+.3f vyaw=%+.3f\n",
        packet_count,
        mode,
        mode_name(mode),
        velocity_x,
        velocity_y,
        velocity_yaw);
  }

  return 0;
}

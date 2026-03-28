// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <chrono>
#include <filesystem>
#include <cstring>
#include <stdexcept>
#include <sys/ioctl.h>
#include <utility>

#include "real_humanoid.h"
#include "motor_controller_conf.h"

namespace
{

  namespace fs = std::filesystem;
  constexpr double kImuReadyTimeoutSeconds = 2.0;
  constexpr double kImuReadyMaxStalenessSeconds = 0.5;

  bool is_workspace_root(const fs::path &candidate)
  {
    return fs::exists(candidate / "pyproject.toml") && fs::exists(candidate / "packages");
  }

  fs::path find_workspace_root(const fs::path &start_path)
  {
    fs::path current = fs::absolute(start_path);
    if (!fs::is_directory(current))
    {
      current = current.parent_path();
    }

    while (!current.empty())
    {
      if (is_workspace_root(current))
      {
        return current;
      }

      const fs::path parent = current.parent_path();
      if (parent == current)
      {
        break;
      }
      current = parent;
    }

    return {};
  }

  fs::path resolve_workspace_path(const fs::path &relative_path)
  {
    if (relative_path.is_absolute())
    {
      return relative_path;
    }

    const fs::path current_working_directory_path = fs::absolute(relative_path);
    if (fs::exists(current_working_directory_path))
    {
      return current_working_directory_path;
    }

    const fs::path workspace_root = find_workspace_root(__FILE__);
    if (!workspace_root.empty())
    {
      return workspace_root / relative_path;
    }

    return current_working_directory_path;
  }

} // namespace

static float linear_interpolate(float start, float end, float percentage)
{
  float target;
  percentage = std::fmin(std::fmax(percentage, 0.0f), 1.0f);
  target = start * (1.f - percentage) + end * percentage;
  return target;
}

RealHumanoid::RealHumanoid(
    const IMUConfiguration &imu_configuration,
    std::string left_leg_bus_name,
    std::string right_leg_bus_name)
    : imu_configuration_(imu_configuration),
      left_leg_bus_name_(std::move(left_leg_bus_name)),
      right_leg_bus_name_(std::move(right_leg_bus_name))
{
  imu = nullptr;
  state = STATE_IDLE;
  next_state = STATE_IDLE;

  for (size_t i = 0; i < N_JOINTS; i += 1)
  {
    position_target[i] = 0;
    position_measured[i] = 0;
    velocity_measured[i] = 0;
    starting_positions[i] = 0;
  }

  for (size_t i = 0; i < N_LOWLEVEL_COMMANDS; i += 1)
  {
    lowlevel_commands[i] = 0;
  }
  for (size_t i = 0; i < N_LOWLEVEL_STATES; i += 1)
  {
    lowlevel_states[i] = 0;
  }

  const fs::path calibration_path = resolve_workspace_path(CALIBRATION_PATH);
  if (fs::exists(calibration_path))
  {
    // calibration.yaml stores offsets derived from the specification-defined
    // calibration reference pose. These offsets are not captured at the
    // farthest mechanical joint limits.
    YAML::Node calibration_config = YAML::LoadFile(calibration_path.string());
    for (size_t i = 0; i < N_JOINTS; i += 1)
    {
      position_offsets[i] = calibration_config["position_offsets"][i].as<float>();
    }
  }
  else
  {
    printf(
        "[INFO] <Main>: Calibration file %s not found, using zero offsets\n",
        calibration_path.string().c_str());
  }

  printf("loaded joint offsets: ");
  for (size_t i = 0; i < N_JOINTS; i += 1)
  {
    printf("%.3f ", position_offsets[i]);
  }
  printf("\n");
  printf(
      "[INFO] <Main>: These joint offsets must come from the calibration reference pose, "
      "not from forcing the robot to its mechanical limits.\n");

  const fs::path policy_config_path = resolve_workspace_path(POLICY_CONFIG_PATH);
  if (!fs::exists(policy_config_path))
  {
    throw std::runtime_error("Unable to find policy config: " + policy_config_path.string());
  }

  YAML::Node policy_config = YAML::LoadFile(policy_config_path.string());
  for (size_t i = 0; i < N_JOINTS; i += 1)
  {
    joint_kp[i] = policy_config["joint_kp"][i].as<float>();
    joint_kd[i] = policy_config["joint_kd"][i].as<float>();
    torque_limit[i] = policy_config["effort_limits"][i].as<float>();
  }
  config_control_dt_ = policy_config["control_dt"].as<float>();
  config_policy_dt_ = policy_config["policy_dt"].as<float>();
}

RealHumanoid::~RealHumanoid()
{
  delete imu;
}

void RealHumanoid::control_loop()
{
  float command_snapshot[N_LOWLEVEL_COMMANDS] = {0};
  ControllerState requested_state = STATE_IDLE;
  float stick_command_velocity_x = 0.0f;
  float stick_command_velocity_y = 0.0f;
  float stick_command_velocity_yaw = 0.0f;
  {
    std::lock_guard<std::mutex> lock(control_input_mutex_);
    requested_state = next_state;
    std::memcpy(command_snapshot, lowlevel_commands, sizeof(command_snapshot));
    stick_command_velocity_x = stick_command_velocity_x_;
    stick_command_velocity_y = stick_command_velocity_y_;
    stick_command_velocity_yaw = stick_command_velocity_yaw_;
  }

  switch (state)
  {
  case STATE_IDLE:
    /* In this state, the motor positions are held at the current measured positions */
    /* When Y is pressed, the robot will switch to RL initialization mode */

    for (int i = 0; i < N_JOINTS; i += 1)
    {
      position_target[i] = position_measured[i];
    }
    if (requested_state == STATE_RL_INIT || requested_state == STATE_RL_RUNNING)
    {
      printf("Switching to RL initialization mode\n");
      state = STATE_RL_INIT;

      for (int i = 0; i < N_JOINTS; i += 1)
      {
        starting_positions[i] = position_target[i];
        usleep(5);
        joint_ptrs[i]->feed();
        joint_ptrs[i]->set_mode(MODE_POSITION);
      }
      init_percentage = 0.0;
    }
    break;

  case STATE_RL_INIT:
    /* In this state, the robot will hold the getup position */
    // printf("init: %.3f\n", init_percentage);

    if (init_percentage < 1.0)
    {
      init_percentage += 1 / 200.0;
      init_percentage = init_percentage < 1.0 ? init_percentage : 1.0;

      for (size_t i = 0; i < N_JOINTS; i += 1)
      {
        position_target[i] = linear_interpolate(starting_positions[i], rl_init_positions[i], init_percentage);
      }
    }
    else
    {
      if (requested_state == STATE_RL_RUNNING)
      {
        printf("Switching to RL running mode\n");
        state = requested_state;
      }
      if (requested_state == STATE_IDLE)
      {
        printf("Switching to idle mode\n");
        state = requested_state;

        for (int i = 0; i < N_JOINTS; i += 1)
        {
          usleep(5);
          joint_ptrs[i]->set_mode(MODE_DAMPING);
        }
      }
    }
    break;

  case STATE_RL_RUNNING:
    /* In this state, the robot will follow the policy */
    for (int i = 0; i < N_LOWLEVEL_COMMANDS; i += 1)
    {
      position_target[i] = command_snapshot[i];
    }

    if (requested_state == STATE_IDLE)
    {
      printf("Switching to idle mode\n");
      state = requested_state;

      for (int i = 0; i < N_JOINTS; i += 1)
      {
        usleep(5);
        joint_ptrs[i]->set_mode(MODE_DAMPING);
      }
    }

    break;
  }

#if DEBUG_DISABLE_TRANSPORTS == 0
  update_joints();
#endif

#if DEBUG_JOINT_DATA_LOGGING == 1
  printf("%d %.2f \t%.2f \t%.2f \t- %.2f \t- %.2f \t%.2f \t",
         control_loop_count,
         position_measured[0], position_measured[1], position_measured[2],
         position_measured[3],
         position_measured[4], position_measured[5]);
  // printf(" %.2f \t%.2f \t%.2f \t- %.2f \t- %.2f \t%.2f",
  //   position_target[0], position_target[1], position_target[2],
  //   position_target[3],
  //   position_target[4], position_target[5]);
  printf("\n");
#endif

  const IMUSnapshot imu_snapshot = imu->snapshot();
  static double last_imu_warning_timestamp = 0.0;
  if (!imu_snapshot.is_ready(kImuReadyMaxStalenessSeconds))
  {
    if (imu_snapshot.captured_at - last_imu_warning_timestamp >= 1.0)
    {
      printf(
          "[WARN] <IMU>: stale or incomplete IMU sample while control is running. "
          "quaternion_ready=%d angular_velocity_ready=%d quaternion_age=%.3f angular_velocity_age=%.3f\n",
          imu_snapshot.quaternion_ready,
          imu_snapshot.angular_velocity_ready,
          imu_snapshot.quaternion_age_seconds(),
          imu_snapshot.angular_velocity_age_seconds());
      last_imu_warning_timestamp = imu_snapshot.captured_at;
    }
  }

  /* base_quat */
  lowlevel_states[0] = imu_snapshot.quaternion[0];
  lowlevel_states[1] = imu_snapshot.quaternion[1];
  lowlevel_states[2] = imu_snapshot.quaternion[2];
  lowlevel_states[3] = imu_snapshot.quaternion[3];

  /* base_ang_vel */
  lowlevel_states[4] = imu_snapshot.angular_velocity[0];
  lowlevel_states[5] = imu_snapshot.angular_velocity[1];
  lowlevel_states[6] = imu_snapshot.angular_velocity[2];

  /* joint_positions */
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    lowlevel_states[7 + i] = position_measured[i];
  }

  /* joint_velocities */
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    lowlevel_states[19 + i] = velocity_measured[i];
  }

  /* mode */
  lowlevel_states[31] = state;

  /* command velocity */
  lowlevel_states[32] = stick_command_velocity_x;
  lowlevel_states[33] = stick_command_velocity_y;
  lowlevel_states[34] = stick_command_velocity_yaw;

  // execute every 4 control loops
  if (control_loop_count >= (int)std::round(config_policy_dt_ / config_control_dt_))
  {
    if (state == STATE_IDLE || state == STATE_RL_RUNNING)
    {
      size_t expected_bytes = sizeof(float) * N_LOWLEVEL_STATES;
      ssize_t actual_bytes = sendto(udp.sockfd, lowlevel_states, expected_bytes, 0, (const struct sockaddr *)&udp.send_addr, sizeof(udp.send_addr));

      if (actual_bytes < 0 || actual_bytes != expected_bytes)
      {
        printf("[Error] <UDP> Error sending: %s\n", strerror(errno));
      }
    }
    size_t expected_bytes = sizeof(float) * N_LOWLEVEL_STATES;
    struct sockaddr_in visualize_addr = {0};
    visualize_addr.sin_family = AF_INET;
    visualize_addr.sin_port = htons(VISUALIZE_PORT);
    visualize_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    ssize_t actual_bytes = sendto(udp.sockfd, lowlevel_states, expected_bytes, 0, (const struct sockaddr *)&visualize_addr, sizeof(visualize_addr));
    control_loop_count = 0;
  }

  control_loop_count += 1;
}

void RealHumanoid::imu_loop()
{
  imu->update_reading();
}

void RealHumanoid::keyboard_loop()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;

  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);

  tcsetattr(0, TCSANOW, &term);

  if (byteswaiting > 0)
  {
    char c = fgetc(stdin);
    printf("key pressed: %c\n", c);

    switch (c)
    {
    case 'r':
      {
        std::lock_guard<std::mutex> lock(control_input_mutex_);
        next_state = STATE_RL_INIT;
      }
      break;
    case 't':
      {
        std::lock_guard<std::mutex> lock(control_input_mutex_);
        next_state = STATE_RL_RUNNING;
      }
      break;
    case 'q':
      {
        std::lock_guard<std::mutex> lock(control_input_mutex_);
        next_state = STATE_IDLE;
      }
      break;
    }
  }

  tcsetattr(0, TCSANOW, &term);
}

void RealHumanoid::joystick_loop()
{
  size_t expected_bytes = 13;
  uint8_t udp_buffer[13];
  ssize_t actual_bytes = recvfrom(udp_joystick.sockfd, udp_buffer, expected_bytes, MSG_DONTWAIT, NULL, NULL);

  if (actual_bytes < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      return;
    }

    static double last_error_timestamp_seconds = 0.0;
    const double now_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now_seconds - last_error_timestamp_seconds >= 1.0)
    {
      printf("[WARN] <UDPStick> Error receiving joystick packet: %s\n", strerror(errno));
      last_error_timestamp_seconds = now_seconds;
    }
    return;
  }

  if (actual_bytes != expected_bytes)
  {
    static double last_size_warning_timestamp_seconds = 0.0;
    const double now_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now_seconds - last_size_warning_timestamp_seconds >= 1.0)
    {
      printf(
          "[WARN] <UDPStick>: Ignoring joystick packet with unexpected size %zd (expected %zu)\n",
          actual_bytes,
          expected_bytes);
      last_size_warning_timestamp_seconds = now_seconds;
    }
    return;
  }

  uint8_t command_mode = udp_buffer[0];
  float stick_command_velocity_x = 0.0f;
  float stick_command_velocity_y = 0.0f;
  float stick_command_velocity_yaw = 0.0f;
  std::memcpy(&stick_command_velocity_x, udp_buffer + 1, sizeof(float));
  std::memcpy(&stick_command_velocity_y, udp_buffer + 5, sizeof(float));
  std::memcpy(&stick_command_velocity_yaw, udp_buffer + 9, sizeof(float));

  {
    std::lock_guard<std::mutex> lock(control_input_mutex_);
    stick_command_velocity_x_ = stick_command_velocity_x;
    stick_command_velocity_y_ = stick_command_velocity_y;
    stick_command_velocity_yaw_ = stick_command_velocity_yaw;

    if (command_mode != 0)
    {
      switch (command_mode)
      {
      case 1:
        next_state = STATE_IDLE;
        break;
      case 2:
        next_state = STATE_RL_INIT;
        break;
      case 3:
        next_state = STATE_RL_RUNNING;
        break;
      default:
        next_state = STATE_IDLE;
      }
    }
  }
}

void RealHumanoid::process_actions()
{
}

void RealHumanoid::process_observations()
{
}

void RealHumanoid::policy_forward()
{
}

void RealHumanoid::udp_recv()
{
  size_t expected_bytes = sizeof(float) * N_LOWLEVEL_COMMANDS;
  float command_buffer[N_LOWLEVEL_COMMANDS] = {0};
  ssize_t actual_bytes = recvfrom(udp.sockfd, command_buffer, expected_bytes, MSG_WAITALL, NULL, NULL);

  if (actual_bytes < 0 || actual_bytes != expected_bytes)
  {
    printf("[Error] <UDP> Error receiving: %s\n", strerror(errno));
    return;
  }

  {
    std::lock_guard<std::mutex> lock(control_input_mutex_);
    std::memcpy(lowlevel_commands, command_buffer, sizeof(command_buffer));
  }
  // TODO: detect if the action is delayed
}

void RealHumanoid::update_joints()
{
  /* set target positions to joint controller */
  for (size_t i = 0; i < N_JOINTS; i += 1)
  {
    joint_ptrs[i]->set_target_position((position_target[i] + position_offsets[i]) * joint_axis_directions[i]);
    // native 低层当前只做位置控制，PDO2 的速度目标保持为零。
    joint_ptrs[i]->set_target_velocity(0.0f);
  }

  joint_ptrs[LEG_LEFT_HIP_ROLL_JOINT]->write_pdo_2();
  joint_ptrs[LEG_RIGHT_HIP_ROLL_JOINT]->write_pdo_2();
  joint_ptrs[LEG_LEFT_HIP_ROLL_JOINT]->read_pdo_2();
  joint_ptrs[LEG_RIGHT_HIP_ROLL_JOINT]->read_pdo_2();

  joint_ptrs[LEG_LEFT_HIP_YAW_JOINT]->write_pdo_2();
  joint_ptrs[LEG_RIGHT_HIP_YAW_JOINT]->write_pdo_2();
  joint_ptrs[LEG_LEFT_HIP_YAW_JOINT]->read_pdo_2();
  joint_ptrs[LEG_RIGHT_HIP_YAW_JOINT]->read_pdo_2();

  joint_ptrs[LEG_LEFT_HIP_PITCH_JOINT]->write_pdo_2();
  joint_ptrs[LEG_RIGHT_HIP_PITCH_JOINT]->write_pdo_2();
  joint_ptrs[LEG_LEFT_HIP_PITCH_JOINT]->read_pdo_2();
  joint_ptrs[LEG_RIGHT_HIP_PITCH_JOINT]->read_pdo_2();

  joint_ptrs[LEG_LEFT_KNEE_PITCH_JOINT]->write_pdo_2();
  joint_ptrs[LEG_RIGHT_KNEE_PITCH_JOINT]->write_pdo_2();
  joint_ptrs[LEG_LEFT_KNEE_PITCH_JOINT]->read_pdo_2();
  joint_ptrs[LEG_RIGHT_KNEE_PITCH_JOINT]->read_pdo_2();

  joint_ptrs[LEG_LEFT_ANKLE_PITCH_JOINT]->write_pdo_2();
  joint_ptrs[LEG_RIGHT_ANKLE_PITCH_JOINT]->write_pdo_2();
  joint_ptrs[LEG_LEFT_ANKLE_PITCH_JOINT]->read_pdo_2();
  joint_ptrs[LEG_RIGHT_ANKLE_PITCH_JOINT]->read_pdo_2();

  joint_ptrs[LEG_LEFT_ANKLE_ROLL_JOINT]->write_pdo_2();
  joint_ptrs[LEG_RIGHT_ANKLE_ROLL_JOINT]->write_pdo_2();
  joint_ptrs[LEG_LEFT_ANKLE_ROLL_JOINT]->read_pdo_2();
  joint_ptrs[LEG_RIGHT_ANKLE_ROLL_JOINT]->read_pdo_2();

  /* update measured positions from joint controller */
  for (size_t i = 0; i < N_JOINTS; i += 1)
  {
    position_measured[i] = joint_ptrs[i]->get_measured_position() * joint_axis_directions[i] - position_offsets[i];
    velocity_measured[i] = joint_ptrs[i]->get_measured_velocity() * joint_axis_directions[i];
  }
}

void RealHumanoid::stop()
{
  if (stopped == 0)
  {
    stopped = 1;

    loop_udp_recv->shutdown();
    loop_control->shutdown();
    loop_keyboard->shutdown();
    loop_joystick->shutdown();

#if DEBUG_DISABLE_TRANSPORTS == 0
    for (int i = 0; i < N_JOINTS; i += 1)
    {
      joint_ptrs[i]->set_mode(MODE_DAMPING);
    }
#endif

    printf("Entered damping mode. Press Ctrl+C again to exit.\n");
  }
  else if (stopped == 1)
  {
    printf("exiting...\n");

#if DEBUG_DISABLE_TRANSPORTS == 0
    for (int i = 0; i < N_JOINTS; i += 1)
    {
      joint_ptrs[i]->set_mode(MODE_IDLE);
    }
#endif

    stopped = 2;

    sleep(1);
  }

  printf("RealHumanoid stopped\n");
}

void RealHumanoid::initialize()
{
  ssize_t status;

#if DEBUG_DISABLE_TRANSPORTS == 0
  // // left arm
  // left_arm_bus.open("can0");
  // if (!left_arm_bus.isOpen()) {
  //   printf("[ERROR] <Main>: Error initializing left arm transport\n");
  //   exit(1);
  // }

  // // right arm
  // right_arm_bus.open("can1");
  // if (!right_arm_bus.isOpen()) {
  //   printf("[ERROR] <Main>: Error initializing right arm transport\n");
  //   exit(1);
  // }

  // left leg
  left_leg_bus.open(left_leg_bus_name_);
  if (!left_leg_bus.isOpen())
  {
    printf(
        "[ERROR] <Main>: Error initializing left leg transport on %s\n",
        left_leg_bus_name_.c_str());
    exit(1);
  }

  // right leg
  right_leg_bus.open(right_leg_bus_name_);
  if (!right_leg_bus.isOpen())
  {
    printf(
        "[ERROR] <Main>: Error initializing right leg transport on %s\n",
        right_leg_bus_name_.c_str());
    exit(1);
  }
#endif

  // // left arm
  // joint_ptrs[ARM_LEFT_SHOULDER_PITCH_JOINT] = std::make_shared<MotorController>(&left_arm_bus, 1);
  // joint_ptrs[ARM_LEFT_SHOULDER_ROLL_JOINT] = std::make_shared<MotorController>(&left_arm_bus, 3);
  // joint_ptrs[ARM_LEFT_SHOULDER_YAW_JOINT] = std::make_shared<MotorController>(&left_arm_bus, 5);
  // joint_ptrs[ARM_LEFT_ELBOW_PITCH_JOINT] = std::make_shared<MotorController>(&left_arm_bus, 7);
  // joint_ptrs[ARM_LEFT_ELBOW_ROLL_JOINT] = std::make_shared<MotorController>(&left_arm_bus, 9);

  // // right arm
  // joint_ptrs[ARM_RIGHT_SHOULDER_PITCH_JOINT] = std::make_shared<MotorController>(&right_arm_bus, 2);
  // joint_ptrs[ARM_RIGHT_SHOULDER_ROLL_JOINT] = std::make_shared<MotorController>(&right_arm_bus, 4);
  // joint_ptrs[ARM_RIGHT_SHOULDER_YAW_JOINT] = std::make_shared<MotorController>(&right_arm_bus, 6);
  // joint_ptrs[ARM_RIGHT_ELBOW_PITCH_JOINT] = std::make_shared<MotorController>(&right_arm_bus, 8);
  // joint_ptrs[ARM_RIGHT_ELBOW_ROLL_JOINT] = std::make_shared<MotorController>(&right_arm_bus, 10);

  // left leg
  joint_ptrs[LEG_LEFT_HIP_ROLL_JOINT] = std::make_shared<MotorController>(&left_leg_bus, 5);
  joint_ptrs[LEG_LEFT_HIP_YAW_JOINT] = std::make_shared<MotorController>(&left_leg_bus, 3);
  joint_ptrs[LEG_LEFT_HIP_PITCH_JOINT] = std::make_shared<MotorController>(&left_leg_bus, 1);
  joint_ptrs[LEG_LEFT_KNEE_PITCH_JOINT] = std::make_shared<MotorController>(&left_leg_bus, 7);
  joint_ptrs[LEG_LEFT_ANKLE_PITCH_JOINT] = std::make_shared<MotorController>(&left_leg_bus, 11);
  joint_ptrs[LEG_LEFT_ANKLE_ROLL_JOINT] = std::make_shared<MotorController>(&left_leg_bus, 13);

  // right leg
  joint_ptrs[LEG_RIGHT_HIP_ROLL_JOINT] = std::make_shared<MotorController>(&right_leg_bus, 6);
  joint_ptrs[LEG_RIGHT_HIP_YAW_JOINT] = std::make_shared<MotorController>(&right_leg_bus, 4);
  joint_ptrs[LEG_RIGHT_HIP_PITCH_JOINT] = std::make_shared<MotorController>(&right_leg_bus, 2);
  joint_ptrs[LEG_RIGHT_KNEE_PITCH_JOINT] = std::make_shared<MotorController>(&right_leg_bus, 8);
  joint_ptrs[LEG_RIGHT_ANKLE_PITCH_JOINT] = std::make_shared<MotorController>(&right_leg_bus, 12);
  joint_ptrs[LEG_RIGHT_ANKLE_ROLL_JOINT] = std::make_shared<MotorController>(&right_leg_bus, 14);

#if DEBUG_DISABLE_TRANSPORTS == 0
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    joint_ptrs[i]->set_mode(MODE_IDLE);
  }
#endif

  imu = new IMU(imu_configuration_);

  status = imu->init();
  if (status < 0)
  {
    printf("[ERROR] <Main>: Error initializing IMU\n");
    exit(1);
  }

  status = initialize_udp(&udp, "0.0.0.0", POLICY_ACS_PORT, ROBOT_IP_ADDR, POLICY_OBS_PORT);
  if (status < 0)
  {
    printf("[ERROR] <Main>: Error initializing UDP\n");
    exit(1);
  }

  // Initialize joystick UDP socket to listen on ROBOT_IP:10011 and send to ROBOT_IP:10011
  status = initialize_udp(&udp_joystick, "0.0.0.0", JOYSTICK_PORT, "127.0.0.1", JOYSTICK_PORT);
  if (status < 0)
  {
    printf("[ERROR] <Main>: Error initializing UDP joystick\n");
    exit(1);
  }

  // 500 Hz UDP receive
  loop_udp_recv = std::make_shared<LoopFunc>("loop_udp_recv", 0.002, [this]
                                             { udp_recv(); }, 1, true, 49);

  // 500 Hz IMU input
  loop_imu = std::make_shared<LoopFunc>("loop_imu", 0.002, [this]
                                        { imu_loop(); }, 1, true, 49);

  // 20 Hz keyboard input
  loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, [this]
                                             { keyboard_loop(); }, 0, true, 19);

  // 20 Hz joystick input
  loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.05, [this]
                                             { joystick_loop(); }, 1, true, 19);

  // we want to ensure all packages are transmitted before killing this thread
  // 100 Hz control loop
  loop_control = std::make_shared<LoopFunc>("loop_control", 0.004, [this]
                                            { control_loop(); }, 0, false, 50, true);
}

void RealHumanoid::run()
{
  initialize();

  printf("Enabling motors...\n");

  printf("read config joint_kp:\n [");
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    printf("%.3f ", joint_kp[i]);
  }
  printf("]\n");
  printf("read config joint_kd:\n [");
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    printf("%.3f ", joint_kd[i]);
  }
  printf("]\n");
  printf("read config torque_limit:\n [");
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    printf("%.3f ", torque_limit[i]);
  }
  printf("]\n");

#if DEBUG_DISABLE_TRANSPORTS == 0
  for (int i = 0; i < N_JOINTS; i += 1)
  {
    usleep(10);
    joint_ptrs[i]->write_position_kp(joint_kp[i]);
    usleep(10);
    joint_ptrs[i]->write_position_kd(joint_kd[i]);
    usleep(10);
    joint_ptrs[i]->write_torque_limit(torque_limit[i]);
    usleep(100);
    joint_ptrs[i]->feed();
    joint_ptrs[i]->set_mode(MODE_DAMPING);
  }
#endif

  printf("Motors enabled\n");

  loop_imu->start();
  if (!imu->wait_until_ready(kImuReadyTimeoutSeconds, kImuReadyMaxStalenessSeconds))
  {
    const IMUSnapshot imu_snapshot = imu->snapshot();
    loop_imu->shutdown();
    throw std::runtime_error(
        "IMU did not become ready before starting native control. timeout=" +
        std::to_string(kImuReadyTimeoutSeconds) +
        " quaternion_ready=" + std::to_string(imu_snapshot.quaternion_ready) +
        " angular_velocity_ready=" + std::to_string(imu_snapshot.angular_velocity_ready) +
        " quaternion_age=" + std::to_string(imu_snapshot.quaternion_age_seconds()) +
        " angular_velocity_age=" + std::to_string(imu_snapshot.angular_velocity_age_seconds()));
  }

  loop_udp_recv->start();
  loop_keyboard->start();
  loop_joystick->start();
  loop_control->start();

  while (stopped != 2)
  {
    sleep(1);
  }
}

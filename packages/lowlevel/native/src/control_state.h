// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

enum ControllerState {
  STATE_ERROR = 0,
  STATE_IDLE = 1,
  STATE_RL_INIT = 2,
  STATE_RL_RUNNING = 3,
  STATE_GETUP = 4,
  STATE_HOLD = 5,
  STATE_GETDOWN = 6,
};

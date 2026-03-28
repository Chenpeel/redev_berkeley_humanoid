#include <cmath>
#include <cstring>
#include <iostream>

#include "motor_controller.h"
#include "motor_controller_conf.h"

namespace {

bool nearly_equal(float lhs, float rhs)
{
  return std::fabs(lhs - rhs) < 1e-6f;
}

bool expect_true(bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << "[FAIL] " << message << std::endl;
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  bool ok = true;

  const uint8_t device_id = 42U;
  const uint32_t receive_pdo_2_can_id = make_can_id(device_id, FUNC_RECEIVE_PDO_2);
  const uint32_t transmit_pdo_2_can_id = make_can_id(device_id, FUNC_TRANSMIT_PDO_2);

  ok &= expect_true(
      get_device_id(receive_pdo_2_can_id) == device_id,
      "make_can_id/get_device_id should round-trip the device id");
  ok &= expect_true(
      get_func_id(receive_pdo_2_can_id) == FUNC_RECEIVE_PDO_2,
      "make_can_id/get_func_id should round-trip the function id");
  ok &= expect_true(
      FUNC_RECEIVE_PDO_2 != FUNC_TRANSMIT_PDO_2,
      "PDO2 transmit and receive function ids must differ");
  ok &= expect_true(
      !matches_can_frame(receive_pdo_2_can_id, device_id, FUNC_TRANSMIT_PDO_2),
      "PDO2 response matching must reject receive frames");
  ok &= expect_true(
      matches_can_frame(transmit_pdo_2_can_id, device_id, FUNC_TRANSMIT_PDO_2),
      "PDO2 response matching must accept matching device/function frames");

  can_frame frame{};
  const float position_target = 1.25f;
  const float velocity_target = 0.0f;
  std::memcpy(frame.data + 0, &position_target, sizeof(float));
  std::memcpy(frame.data + sizeof(float), &velocity_target, sizeof(float));

  float decoded_position_target = 0.0f;
  float decoded_velocity_target = 0.0f;
  std::memcpy(&decoded_position_target, frame.data + 0, sizeof(float));
  std::memcpy(&decoded_velocity_target, frame.data + sizeof(float), sizeof(float));

  ok &= expect_true(
      nearly_equal(decoded_position_target, position_target),
      "PDO2 payload should encode position target in the first float");
  ok &= expect_true(
      nearly_equal(decoded_velocity_target, velocity_target),
      "PDO2 payload should encode velocity target in the second float");

  ok &= expect_true(MODE_IDLE != MODE_DAMPING, "MODE_IDLE and MODE_DAMPING must differ");
  ok &= expect_true(MODE_IDLE != MODE_POSITION, "MODE_IDLE and MODE_POSITION must differ");
  ok &= expect_true(MODE_DAMPING != MODE_POSITION, "MODE_DAMPING and MODE_POSITION must differ");

  if (!ok)
  {
    return 1;
  }

  std::cout << "test-lowlevel-control passed" << std::endl;
  return 0;
}

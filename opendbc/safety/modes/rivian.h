#pragma once

#include "opendbc/safety/declarations.h"

// Rivian Primary Actuator CAN (merged DBC) - angle-based steering
// TX: 0x110 ACM_SteeringControl (ACM_SteeringAngleRequest in deg)
// RX: 0x40 SAS_Status (steering angle sensor), 0x152 VDM_OutputSignals (vehicle speed, EPAS mode)

#define RIVIAN_ACM_STEERING_CONTROL 0x110U
#define RIVIAN_SAS_STATUS           0x40U
#define RIVIAN_VDM_OUTPUT_SIGNALS   0x152U

static safety_config rivian_init(uint16_t param) {
  static const CanMsg RIVIAN_STOCK_TX_MSGS[] = {
    {RIVIAN_ACM_STEERING_CONTROL, 0, 8, .check_relay = true},
  };

  static RxCheck rivian_rx_checks[] = {
    {.msg = {{RIVIAN_SAS_STATUS, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{RIVIAN_VDM_OUTPUT_SIGNALS, 0, 8, 50U, .ignore_checksum = true, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  SAFETY_UNUSED(param);
  return BUILD_SAFETY_CFG(rivian_rx_checks, RIVIAN_STOCK_TX_MSGS);
}

static void rivian_rx_hook(const CANPacket_t *msg) {
  if (msg->bus != 0U) {
    return;
  }

  if (msg->addr == RIVIAN_SAS_STATUS) {
    // SAS_Status_AngleSafe : 23|15@0- (0.0009765625, 0) rad -> convert to deg for angle_meas
    // 15 bits signed, start bit 23, little-endian: bit 23 in byte 2, bits 24-31 byte 3, bits 32-37 byte 4
    int raw = ((msg->data[2] & 0x80U) >> 7) << 14 | (msg->data[3] << 6) | (msg->data[4] >> 2);
    raw = to_signed(raw, 15);
    // 0.0009765625 rad/bit -> deg
    int angle_deg = ROUND((float)raw * 0.0009765625f * 57.2957795f);
    update_sample(&angle_meas, angle_deg);
  }

  if (msg->addr == RIVIAN_VDM_OUTPUT_SIGNALS) {
    // VDM_VehicleSpeed : 47|16@0+ (0.01, 0) kph -> m/s for UPDATE_VEHICLE_SPEED
    int speed_raw = (msg->data[5] >> 7) | (msg->data[6] << 1) | ((msg->data[7] & 0x7FU) << 9);
    UPDATE_VEHICLE_SPEED((float)speed_raw * 0.01f / 3.6f);
    // VDM_EpasPowerMode in byte 7 bits 4-5: 1 = Drive_On (allow controls)
    int epas_mode = (msg->data[7] >> 4) & 0x3U;
    if (epas_mode != 1U) {
      controls_allowed = false;
    }
  }
}

static bool rivian_tx_hook(const CANPacket_t *msg) {
  if (msg->addr == RIVIAN_ACM_STEERING_CONTROL) {
    // ACM_SteeringAngleRequest : 23|15@0+ (0.1, -1638.4) deg
    int raw = ((msg->data[2] & 0x80U) >> 7) << 14 | (msg->data[3] << 6) | (msg->data[4] >> 2);
    raw &= 0x7FFFU;
    int desired_angle = (int)((float)raw * 0.1f - 1638.4f);

    // ACM_EacEnabled in bits 13-14: 2 = EAC enabled
    bool steer_control_enabled = ((msg->data[1] >> 5) & 0x3U) == 2U;

    // Angle limits from merged DBC: ±1638.3 deg theoretical; use ±600 deg for steering wheel
    const struct lookup_t RIVIAN_ANGLE_RATE_UP = {{0.f, 0.f, 0.f}, {100.f, 100.f, 100.f}};
    const struct lookup_t RIVIAN_ANGLE_RATE_DOWN = {{0.f, 0.f, 0.f}, {100.f, 100.f, 100.f}};
    const AngleSteeringLimits RIVIAN_STEERING_LIMITS = {
      .max_angle = 600,
      .angle_deg_to_can = 10.f,
      .angle_rate_up_lookup = RIVIAN_ANGLE_RATE_UP,
      .angle_rate_down_lookup = RIVIAN_ANGLE_RATE_DOWN,
      .max_angle_error = 90,
      .angle_error_min_speed = 1.f,
      .frequency = 100U,
      .angle_is_curvature = false,
      .enforce_angle_error = true,
      .inactive_angle_is_zero = false,
    };

    if (!steer_angle_cmd_checks(desired_angle, steer_control_enabled, RIVIAN_STEERING_LIMITS)) {
      return false;
    }
  }
  return true;
}

const safety_hooks rivian_hooks = {
  .init = rivian_init,
  .rx = rivian_rx_hook,
  .tx = rivian_tx_hook,
  .get_counter = NULL,
  .get_checksum = NULL,
  .compute_checksum = NULL,
};

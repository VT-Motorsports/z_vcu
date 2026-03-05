// logger_encoders.h
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include "vehicle_state.h"

// ============================================================================
// VCU Logger CAN Encoders — custom telemetry frames transmitted on CAN2
//
// Frame format uses big-endian byte order to match DTI encoder convention.
// Scaled integer encoding avoids floating-point on the receiver side.
//
// Usage:
//   struct can_frame f{};
//   encode_apps_state(&f, vehicle());
//   hardware_->can2.send(&f, K_NO_WAIT);
//
// TX CAN IDs (CAN2):
//   APPS state (0x100): pedal1, pedal2, commanded torque, error flags
// ============================================================================

// --- APPS state (ID 0x100, DLC 8) ---
// Byte 0-1: pedal1_percent    × 10000, uint16_t big-endian  (0 = 0.00%, 10000 = 100.00%)
// Byte 2-3: pedal2_percent    × 10000, uint16_t big-endian
// Byte 4-5: commandedTorque   × 10000, uint16_t big-endian
// Byte 6:   error bitfield — bit i set when APPSIf.errors[i] is true (APPS_ERRORS enum order)
// Byte 7:   0x00 reserved
void encode_apps_state(struct can_frame *frame, const volatile VehicleState *vd);

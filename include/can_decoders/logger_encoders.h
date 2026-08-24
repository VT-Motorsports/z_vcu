// logger_encoders.h
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include "vehicle_state.h"

// ============================================================================
// VCU Logger CAN Encoders — custom telemetry frames transmitted on CAN1
//
// Frame format uses big-endian byte order to match DTI encoder convention.
// Scaled integer encoding avoids floating-point on the receiver side.
//
// Usage:
//   struct can_frame f{};
//   encode_apps_state(&f, vehicle());
//   hardware_->can1.send(&f, K_NO_WAIT);
//
// TX CAN IDs (CAN1):
//   APPS state (0x100): pedal1, pedal2, commanded torque, error flags
// ============================================================================

// --- APPS state (ID 0x100, DLC 8) ---
// Byte 0-1: pedal1_percent    × 10000, uint16_t big-endian  (0 = 0.00%, 10000 = 100.00%)
// Byte 2-3: pedal2_percent    × 10000, uint16_t big-endian
// Byte 4-5: commandedTorque   × 10000, uint16_t big-endian
// Byte 6:   error bitfield — bit i set when APPSIf.errors[i] is true (APPS_ERRORS enum order)
// Byte 7:   0x00 reserved
void encode_apps_state(struct can_frame *frame, const volatile VehicleState *vd);

// --- ADC analog channels (ID 0x200, 0x201, DLC 8, four channels per frame) ---
// Scale: voltage = channels[i] / 65535.0f * 5.0f, transmitted as uint16 * 100
// Range: 0–500 (0.00V–5.00V), resolution: 0.01V
void encode_analog_channels(struct can_frame frames[2], const volatile VehicleState *vd);

// --- ADC raw channels (ID 0x202, 0x203, DLC 8, four channels per frame) ---
// Raw 16-bit ADC counts directly from hardware, no scaling applied.
// Range: 0–65535 counts (0.00V–5.00V at 12-bit/16-bit resolution)
void encode_analog_channels_raw(struct can_frame frames[2], const volatile VehicleState *vd);

// --- VSM fault vector (ID 0x101, DLC 8) ---
// Byte 0-7: 64-bit fault bitfield, big-endian
//           Bit index matches VSM_FAULTS enum value.
void encode_vsm_faults(struct can_frame *frame, const volatile VehicleState *vd);

// --- VSM state (ID 0x102, DLC  8) ---
// Byte 0:   state enum (VSM state machine: POST/Precharge/Armed/Drive/Fault)
// Byte 1:   flags — bit 0: faulted, bit 1: RTDS active, bits 2-7: reserved
// Byte 2-5: precharge elapsed time (ms), uint32 big-endian (0 if not precharging)
// Byte 6-7: RTDS elapsed time (ms), uint16 big-endian (0 if not sounding)
void encode_vsm_state(struct can_frame *frame, const volatile VehicleState *vd);

// --- VSM bus telemetry (ID 0x103, DLC 8) ---
// Byte 0-1: dc_link_voltage × 10, uint16 big-endian (0.0-6553.5 V, 0.1V resolution)
// Byte 2-3: estimated_link_voltage × 10, uint16 big-endian
// Byte 4-7: inverter_current_summed × 100, int32 big-endian (±21474836 A, 0.01A resolution)
void encode_vsm_telemetry(struct can_frame *frame, const volatile VehicleState *vd);
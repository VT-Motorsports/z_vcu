// dti_encoders.h
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include "vehicle_state.h"

// ============================================================================
// Manual: DTI CAN Manual V2.5 — Standard ID (11-bit) format
// CAN ID = (packet_id << 5) | node_id  (same formula as receive)
// Node IDs:  FL=22  FR=23  RL=24  RR=25
//
// Encoder signature is the exact inverse of the decoder:
//   Decoder: void decode_dti_fl_0x1F(const struct can_frame *,       volatile VehicleState *)
//   Encoder: void encode_dti_fl_*(   struct can_frame *,        const volatile VehicleState *)
//
// Usage:
//   struct can_frame f{};
//   encode_dti_fl_set_ac_current(&f, vehicle());
//   hardware_->can1.send(&f, K_NO_WAIT);
//
// TX CAN IDs:
//   SetAcCurrent   (packet 0x01): FL=0x036  FR=0x037  RL=0x038  RR=0x039
//   SetDriveEnable (packet 0x0C): FL=0x196  FR=0x197  RL=0x198  RR=0x199
// ============================================================================

// --- SetAcCurrent (packet 0x01) ---
// Packs INVERTERS[c].cmd_ac_current into an 8-byte frame.
// cmd_ac_current: A_pk × 10, signed. Positive = motoring, negative = regen.
void encode_dti_fl_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd);
void encode_dti_fr_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd);
void encode_dti_rl_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd);
void encode_dti_rr_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd);

// --- SetDriveEnable (packet 0x0C) ---
// Packs INVERTERS[c].cmd_drive_enable into a 2-byte frame.
// cmd_drive_enable: 0 = disabled, 1 = enabled.
void encode_dti_fl_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd);
void encode_dti_fr_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd);
void encode_dti_rl_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd);
void encode_dti_rr_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd);

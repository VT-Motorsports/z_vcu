// dti_encoders.cpp
//
// DTI HV-500/550/850 CAN2 Encoders
// Reference: DTI CAN Manual V2.5
// Byte order: Big Endian (Motorola), unused bytes = 0x00

#include "can_decoders/dti_encoders.h"
#include <string.h>

// ============================================================================
// Internal helpers
// ============================================================================

static const uint8_t kNodeId[NUM_CORNERS] = {22, 23, 24, 25}; // FL FR RL RR

// Inverse of be16() in dti_decoders.cpp — packs a signed 16-bit value big-endian
static inline void put_be16(uint8_t *d, int16_t v)
{
    d[0] = (uint8_t)((uint16_t)v >> 8);
    d[1] = (uint8_t)v;
}

// --- packet 0x01: SetAcCurrent ---
// DLC 8, signal at d[0..1] (MSB first), d[2..7] = 0x00
static void encode_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd, Corner c)
{
    frame->id    = (0x01u << 5) | kNodeId[c];
    frame->dlc   = 8;
    frame->flags = 0;
    memset(frame->data, 0x00, 8);
    put_be16(&frame->data[0], vd->INVERTERS[c].cmd_ac_current);
}

// --- packet 0x0C: SetDriveEnable ---
// DLC 2, d[0] = enable byte (0x00 or 0x01), d[1] = 0x00
static void encode_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd, Corner c)
{
    frame->id    = (0x0Cu << 5) | kNodeId[c];
    frame->dlc   = 2;
    frame->flags = 0;
    frame->data[0] = vd->INVERTERS[c].cmd_drive_enable;
    frame->data[1] = 0x00;
}

// ============================================================================
// Public per-corner wrappers — SetAcCurrent (packet 0x01)
// ============================================================================

void encode_dti_fl_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_ac_current(frame, vd, FRONT_LEFT);
}

void encode_dti_fr_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_ac_current(frame, vd, FRONT_RIGHT);
}

void encode_dti_rl_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_ac_current(frame, vd, REAR_LEFT);
}

void encode_dti_rr_set_ac_current(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_ac_current(frame, vd, REAR_RIGHT);
}

// ============================================================================
// Public per-corner wrappers — SetDriveEnable (packet 0x0C)
// ============================================================================

void encode_dti_fl_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_drive_enable(frame, vd, FRONT_LEFT);
}

void encode_dti_fr_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_drive_enable(frame, vd, FRONT_RIGHT);
}

void encode_dti_rl_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_drive_enable(frame, vd, REAR_LEFT);
}

void encode_dti_rr_set_drive_enable(struct can_frame *frame, const volatile VehicleState *vd)
{
    encode_set_drive_enable(frame, vd, REAR_RIGHT);
}

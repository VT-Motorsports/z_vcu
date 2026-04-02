// orion_bms_decoders.h
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include "vehicle_state.h"

// ============================================================================
// Reference: Orion BMS2 — Standard ID (11-bit) format
// All message IDs are fixed (no node-ID encoding).
//
// Registration (CAN2):
//   bus_handlers[0x200] = decode_orion_0x200;  // Cell broadcast
//   bus_handlers[0x3E8] = decode_orion_0x3E8;  // Low cell stats
//   bus_handlers[0x3E9] = decode_orion_0x3E9;  // High cell stats
//   bus_handlers[0x3F0] = decode_orion_0x3F0;  // Pack extended
//   bus_handlers[0x3F1] = decode_orion_0x3F1;  // Resistance / avg cell
//   bus_handlers[0x3F2] = decode_orion_0x3F2;  // Adaptive capacity
//   bus_handlers[0x3F3] = decode_orion_0x3F3;  // DTC flags
//   bus_handlers[0x3F4] = decode_orion_0x3F4;  // DCL / limit status
//   bus_handlers[0x3F5] = decode_orion_0x3F5;  // Pack current (fast)
//   bus_handlers[0x6B0] = decode_orion_0x6B0;  // Pack status
//   bus_handlers[0x6B1] = decode_orion_0x6B1;  // Temps / limits
//
// Max ID = 0x6B1 (1713) — fits in 2048 table
// ============================================================================

// --- 0x6B0: Pack Current, Voltage, SOC, Relay/IO flags (8 ms) ---
void decode_orion_0x6B0(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x6B1: Pack DCL, CCL, High/Low Temperature (104 ms) ---
void decode_orion_0x6B1(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3E8: Low Cell Voltage, Open-cell Voltage, Resistance (104 ms) ---
void decode_orion_0x3E8(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3E9: High Cell Voltage, Open-cell ID, Resistance (1008 ms) ---
void decode_orion_0x3E9(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3F0: Pack SOC, Inst Voltage, Open Voltage, Current (96 ms) ---
void decode_orion_0x3F0(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3F1: Pack Resistance, Average Cell Voltage (1008 ms) ---
void decode_orion_0x3F1(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3F2: Adaptive Total Capacity, Amphours, SOC (1000 ms) ---
void decode_orion_0x3F2(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3F3: DTC Flags 1 & 2 (1000 ms) ---
void decode_orion_0x3F3(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3F4: Pack DCL, Current Limits Status (1008 ms) ---
void decode_orion_0x3F4(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x3F5: Pack Current fast (8 ms) ---
void decode_orion_0x3F5(const struct can_frame *frame, volatile VehicleState *vd);

// --- 0x200: Cell Broadcast — round-robin per cell (12 ms) ---
void decode_orion_0x200(const struct can_frame *frame, volatile VehicleState *vd);
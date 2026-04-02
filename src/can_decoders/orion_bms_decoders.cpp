// orion_bms_decoders.cpp
//
// Orion BMSIf2 CAN Decoders
// Reference: Orion_CANBUS_VCU_2023_CAR.dbc
// Byte order: Big Endian (Motorola), unused = 0xFF
// All signals stored as raw integers; apply DBC scale/offset at point of use.

#include "can_decoders/orion_bms_decoders.h"
#include "vehicle_state.h"

// ============================================================================
// Internal helpers
// ============================================================================

static inline int16_t be16(const uint8_t *d)
{
    return (int16_t)(((uint16_t)d[0] << 8) | d[1]);
}

// ============================================================================
// Decoders — one per message ID, plugs directly into CAN dispatch table
// ============================================================================

// --- 0x6B0: Pack Current, Pack Voltage, SOC, Relay/IO status ---
// Transmit rate: 8 ms
void decode_orion_0x6B0(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.pack_current = be16(&d[0]);
    bms.pack_inst_voltage = (uint16_t)be16(&d[2]);
    bms.pack_soc = d[4];

    // Byte 5: I/O flags
    uint8_t io0 = d[5];
    bms.multipurpose_input_2 = (io0 >> 0) & 1;
    bms.multipurpose_input_3 = (io0 >> 1) & 1;
    bms.multipurpose_output_2 = (io0 >> 3) & 1;
    bms.multipurpose_output_3 = (io0 >> 4) & 1;
    bms.multipurpose_output_4 = (io0 >> 5) & 1;
    bms.multipurpose_enable = (io0 >> 6) & 1;
    bms.multipurpose_output = (io0 >> 7) & 1;

    // Byte 6: relay flags
    uint8_t relay = d[6];
    bms.discharge_relay = (relay >> 0) & 1;
    bms.charge_relay = (relay >> 1) & 1;
    bms.charger_safety = (relay >> 2) & 1;
    bms.error_mil_output = (relay >> 3) & 1;
    bms.multipurpose_input = (relay >> 4) & 1;
    bms.constant_1 = (relay >> 5) & 1;
    bms.ready_power_signal = (relay >> 6) & 1;
    bms.charge_power_signal = (relay >> 7) & 1;

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x6B1: Pack DCL, Pack CCL, High/Low Temperature ---
// Transmit rate: 104 ms
void decode_orion_0x6B1(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.pack_dcl = (uint16_t)be16(&d[0]);
    bms.pack_ccl = d[2];
    bms.high_temperature = (int8_t)d[4];
    bms.low_temperature = (int8_t)d[5];

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3E8: Low Cell Voltage ID, Voltage, Open-cell Voltage, Resistance ---
// Transmit rate: 104 ms
void decode_orion_0x3E8(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.low_cell_voltage_id = d[0];
    bms.low_cell_voltage = (uint16_t)be16(&d[1]);
    bms.low_opencell_voltage = (uint16_t)be16(&d[3]);
    bms.low_cell_resistance = (uint16_t)be16(&d[5]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3E9: High Cell Voltage ID, Voltage, Open-cell ID, Resistance ---
// Transmit rate: 1008 ms
void decode_orion_0x3E9(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.high_cell_voltage_id = d[0];
    bms.high_cell_voltage = (uint16_t)be16(&d[1]);
    bms.high_opencell_id = (uint16_t)be16(&d[3]);
    bms.high_cell_resistance = (uint16_t)be16(&d[5]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3F0: Pack SOC, Instantaneous Voltage, Open Voltage, Current ---
// Transmit rate: 96 ms
void decode_orion_0x3F0(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.pack_soc_ext = d[0];
    bms.pack_inst_voltage_ext = (uint16_t)be16(&d[1]);
    bms.pack_open_voltage = (uint16_t)be16(&d[3]);
    bms.pack_current_ext = (uint16_t)be16(&d[5]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3F1: Pack Resistance, Average Cell Voltage ---
// Transmit rate: 1008 ms
void decode_orion_0x3F1(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.pack_resistance = (uint16_t)be16(&d[0]);
    bms.avg_cell_voltage = (uint16_t)be16(&d[2]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3F2: Adaptive Total Capacity, Adaptive Amphours, Adaptive SOC ---
// Transmit rate: 1000 ms
void decode_orion_0x3F2(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.adaptive_total_capacity = (uint16_t)be16(&d[0]);
    bms.adaptive_amphours = (uint16_t)be16(&d[2]);
    bms.adaptive_soc = d[4];

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3F3: DTC Flags ---
// Transmit rate: 1000 ms
void decode_orion_0x3F3(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.dtc_flags_1 = (uint16_t)be16(&d[0]);
    bms.dtc_flags_2 = (uint16_t)be16(&d[2]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3F4: Pack DCL, Current Limits Status ---
// Transmit rate: 1008 ms
void decode_orion_0x3F4(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.pack_dcl_ext = (uint16_t)be16(&d[0]);
    bms.current_limits_status = (uint16_t)be16(&d[2]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x3F5: Pack Current (fast) ---
// Transmit rate: 8 ms
void decode_orion_0x3F5(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    bms.pack_current_fast = (uint16_t)be16(&d[0]);

    bms.last_rx_time_ms = k_uptime_get();
}

// --- 0x200: Cell Broadcast (round-robin, one cell per frame) ---
// Transmit rate: 12 ms
//   d[0]     CellId
//   d[1..2]  CellVoltage       (16-bit, × 0.0001 V)
//   d[3]     bit 7: CellBalancing, bits [6:0]: CellResistance upper 7
//   d[4]     CellResistance lower 8  (15-bit total, × 0.01 mOhm)
//   d[5..6]  CellOpenVoltage   (16-bit, × 0.0001 V)
//   d[7]     Checksum = (0x200 + 8 + d[0..6]) & 0xFF
void decode_orion_0x200(const struct can_frame *f, volatile VehicleState *vd)
{
    volatile auto &bms = vd->BMSIf;
    const uint8_t *d = f->data;

    uint8_t cell_id = d[0];
    if (cell_id >= BMS_data::NUM_CELLS)
    {
        return;
    }

    bms.cells[cell_id].voltage = (uint16_t)be16(&d[1]);
    bms.cells[cell_id].balancing = (d[3] >> 7) & 1;
    bms.cells[cell_id].resistance = (uint16_t)(((d[3] & 0x7F) << 8) | d[4]);
    bms.cells[cell_id].open_voltage = (uint16_t)be16(&d[5]);

    bms.last_cell_rx_time_ms = k_uptime_get();
}
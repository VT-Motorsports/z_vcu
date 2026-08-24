// logger_encoders.cpp
//
// VCU telemetry CAN encoders for CAN2.
// Byte order: Big Endian (Motorola), unused bytes = 0x00.

#include "can_decoders/logger_encoders.h"
#include <cstdint>
#include <string.h>
#include <utility>

static inline void put_be16(uint8_t *d, uint16_t v) {
    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)v;
}

// --- APPS state (ID 0x100, DLC 8) ---
void encode_apps_state(struct can_frame *frame, const volatile VehicleState *vd) {
    frame->id = 0x100u;
    frame->dlc = 8;
    frame->flags = 0;
    memset(frame->data, 0x00, 8);

    put_be16(&frame->data[0], (uint16_t)(vd->APPSIf.pedal1_percent * 10000.0f));
    put_be16(&frame->data[2], (uint16_t)(vd->APPSIf.pedal2_percent * 10000.0f));
    put_be16(&frame->data[4], (uint16_t)(vd->APPSIf.commandedTorquePercentage * 10000.0f));

    uint8_t err_flags = 0;
    for (int i = 0; i < APPS_ERRORS::NUM_ERRORS; ++i) {
        if (vd->APPSIf.errors[i]) {
            err_flags |= (1u << i);
        }
    }
    frame->data[6] = err_flags;
}

void encode_analog_channels(struct can_frame frames[2], const volatile VehicleState *vd) {
    for (int f = 0; f < 2; f++) {
        frames[f].id = 0x200u + f;
        frames[f].dlc = 8;
        frames[f].flags = 0;
        memset(frames[f].data, 0x00, sizeof(frames[f].data));

        for (int i = 0; i < 4; i++) {
            int ch = f * 4 + i;
            float v = vd->analogIf.get_voltage(ch);
            if (v < 0.0f)
                v = 0.0f;
            if (v > 655.35f)
                v = 655.35f;
            uint16_t raw = (uint16_t)(v * 100.0f);
            put_be16(&frames[f].data[i * 2], raw);
        }
    }
}

void encode_analog_channels_raw(struct can_frame frames[2], const volatile VehicleState *vd) {
    for (int f = 0; f < 2; f++) {
        frames[f].id = 0x202u + f;
        frames[f].dlc = 8;
        frames[f].flags = 0;
        memset(frames[f].data, 0x00, sizeof(frames[f].data));

        for (int i = 0; i < 4; i++) {
            int ch = f * 4 + i;
            uint16_t raw = vd->analogIf.channels[ch];
            put_be16(&frames[f].data[i * 2], raw);
        }
    }
}

void encode_vsm_faults(struct can_frame *frame, const volatile VehicleState *vd) {
    frame->id = 0x101u;
    frame->dlc = 8;
    frame->flags = 0;
    memset(frame->data, 0x00, 8);

    uint64_t faults = vd->VSM_If->FAULTS.to_ullong();
    for (int i = 0; i < 8; i++) {
        frame->data[i] = (uint8_t)(faults >> (56 - i * 8));
    }
}

void encode_vsm_state(struct can_frame *frame, const volatile VehicleState *vd) {
    frame->id = 0x102u;
    frame->dlc = 8;
    frame->flags = 0;
    memset(frame->data, 0x00, 8);

    // Byte 0: state machine state (adjust member name to match your VSM)
    frame->data[0] = std::to_underlying(vd->VSM_STATE->load());

    // Byte 2-5: precharge elapsed time (ms)
    uint32_t precharge_elapsed = 0;
    if (vd->VSM_If->precharging_start_time != 0) {
        int64_t now = k_uptime_get();
        int64_t delta = now - vd->VSM_If->precharging_start_time;
        if (delta < 0)
            delta = 0;
        if (delta > UINT32_MAX)
            delta = UINT32_MAX;
        precharge_elapsed = (uint32_t)delta;
    }
    frame->data[2] = (uint8_t)(precharge_elapsed >> 24);
    frame->data[3] = (uint8_t)(precharge_elapsed >> 16);
    frame->data[4] = (uint8_t)(precharge_elapsed >> 8);
    frame->data[5] = (uint8_t)(precharge_elapsed);

    // Byte 6-7: RTDS elapsed time (ms)
    uint16_t rtds_elapsed = 0;
    if (vd->VSM_If->RTDS_start_time != 0) {
        int64_t now = k_uptime_get();
        int64_t delta = now - vd->VSM_If->RTDS_start_time;
        if (delta < 0)
            delta = 0;
        if (delta > UINT16_MAX)
            delta = UINT16_MAX;
        rtds_elapsed = (uint16_t)delta;
    }
    put_be16(&frame->data[6], rtds_elapsed);
}

void encode_vsm_telemetry(struct can_frame *frame, const volatile VehicleState *vd) {
    frame->id = 0x103u;
    frame->dlc = 8;
    frame->flags = 0;
    memset(frame->data, 0x00, 8);

    // Clamp helpers
    auto clamp_u16_voltage = [](float v) -> uint16_t {
        v *= 10.0f;
        if (v < 0.0f)
            return 0;
        if (v > 65535.0f)
            return 65535;
        return (uint16_t)v;
    };

    put_be16(&frame->data[0], clamp_u16_voltage(vd->VSM_If->dc_link_voltage));
    put_be16(&frame->data[2], clamp_u16_voltage(vd->VSM_If->estimated_link_voltage));

    // inverter_current_summed is std::atomic<float> — load it, then scale.
    float current = vd->VSM_If->inverter_current_summed.load(std::memory_order_relaxed);
    float scaled = current * 100.0f;
    int32_t current_raw;
    if (scaled > (float)INT32_MAX)
        current_raw = INT32_MAX;
    else if (scaled < (float)INT32_MIN)
        current_raw = INT32_MIN;
    else
        current_raw = (int32_t)scaled;

    frame->data[4] = (uint8_t)((uint32_t)current_raw >> 24);
    frame->data[5] = (uint8_t)((uint32_t)current_raw >> 16);
    frame->data[6] = (uint8_t)((uint32_t)current_raw >> 8);
    frame->data[7] = (uint8_t)((uint32_t)current_raw);
}
// logger_encoders.cpp
//
// VCU telemetry CAN encoders for CAN2.
// Byte order: Big Endian (Motorola), unused bytes = 0x00.

#include "can_decoders/logger_encoders.h"
#include <string.h>

static inline void put_be16(uint8_t *d, uint16_t v)
{
    d[0] = (uint8_t)(v >> 8);
    d[1] = (uint8_t)v;
}

// --- APPS state (ID 0x100, DLC 8) ---
void encode_apps_state(struct can_frame *frame, const volatile VehicleState *vd)
{
    frame->id    = 0x100u;
    frame->dlc   = 8;
    frame->flags = 0;
    memset(frame->data, 0x00, 8);

    put_be16(&frame->data[0], (uint16_t)(vd->APPSIf.pedal1_percent         * 10000.0f));
    put_be16(&frame->data[2], (uint16_t)(vd->APPSIf.pedal2_percent         * 10000.0f));
    put_be16(&frame->data[4], (uint16_t)(vd->APPSIf.commandedTorquePercentage * 10000.0f));

    uint8_t err_flags = 0;
    for (int i = 0; i < APPS_ERRORS::NUM_ERRORS; ++i) {
        if (vd->APPSIf.errors[i]) {
            err_flags |= (1u << i);
        }
    }
    frame->data[6] = err_flags;
}

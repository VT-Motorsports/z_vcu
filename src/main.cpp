#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "threads/APPS.h"
#include "hardware.h"
#include "threads/Logger.h"
#include "threads/VSM_task.h"
#include "threads/system.h"
#include "vehicle_state.h"

LOG_MODULE_REGISTER(main);

// Pedal ADC: 3V = 0% throttle, 2V = 100% throttle
#define PEDAL_V_MIN 3.00f
#define PEDAL_V_MAX 2.00f
#define MAX_AC_CURRENT 10

// DTI Standard CAN ID: (packet_id << 5) | node_id
#define DTI_NODE_ID 25
#define DTI_STD_ID(pkt) (((pkt) << 5) | DTI_NODE_ID)

// DTI Packet IDs (commands)
#define PKT_SET_AC_CUR 0x01
#define PKT_DRIVE_ENABLE 0x0C

int main(void) {
    LOG_INF("***VCU ENTERED MAIN***");

    static VehicleState vehicle;
    static Hardware hardware(&vehicle);
    static System system;

    LOG_INF("=== VCU Starting ===");

    if (system.init() != 0) {
        LOG_ERR("System init failed!");
        return -1;
    }

    if (hardware.init() != 0) {
        LOG_ERR("Hardware init failed!");
        return -2;
    }

    start_apps_task(&vehicle, &hardware);
    start_diagnostics_task(&system, &hardware, &vehicle);
    start_logger_task(&system, &hardware, &vehicle);
    start_VSM_task(&system, &hardware, &vehicle);

    LOG_INF("=== VCU Ready ===");

    while (1) {
        k_sleep(K_MSEC(10000));
    }
}

/*
while (1) {
        float pedal_v = hardware.adc_chan0.read_voltage();

        // Inverted mapping: 3V -> 0%, 2V -> 100%
        float pedal_pct = (pedal_v - PEDAL_V_MIN) / (PEDAL_V_MAX - PEDAL_V_MIN);
        pedal_pct = CLAMP(pedal_pct, 0.0f, 1.0f);

        // DTI expects value * 10, big-endian 2 bytes
        int16_t ac_scaled = static_cast<int16_t>(pedal_pct * MAX_AC_CURRENT * 10);

        // Drive enable (packet 0x0C) -> CAN ID 0x196
        struct can_frame drive_en = {.id = DTI_STD_ID(PKT_DRIVE_ENABLE),
                                     .dlc = 8,

                                     .flags = 0,
                                     .data = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

        // Set AC current (packet 0x01) -> CAN ID 0x36
        struct can_frame ac_cmd = {.id = DTI_STD_ID(PKT_SET_AC_CUR),
                                   .dlc = 8,

                                   .flags = 0,
                                   .data = {static_cast<uint8_t>((ac_scaled >> 8) & 0xFF),
                                            static_cast<uint8_t>(ac_scaled & 0xFF), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                            0xFF}};

        bool drive_state;
        hardware.drive_enable.get(&drive_state);
        if (drive_state) {
            drive_en.data[0] = 0x00;
        } else {
            drive_en.data[0] = 0x01;
        }

        int result = hardware.can1.send(&drive_en, K_NO_WAIT);

        result = hardware.can1.send(&ac_cmd, K_NO_WAIT);

        LOG_INF("Pedal: %.2fV  %d%%  AC: %dA", static_cast<double>(pedal_v), static_cast<int>(pedal_pct * 100),
                ac_scaled / 10);

        k_msleep(100);
    }*/
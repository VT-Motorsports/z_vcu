// vehicle_state.h
#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

enum Corner : uint8_t
{
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    NUM_CORNERS = 4
};

struct DTI_Inverter
{
    uint8_t node_id;

    // Packet 0x1F: General Data 6
    uint8_t control_mode;    // 1=Speed,2=Current,3=CurrBrake,4=Pos,7=None
    int16_t target_iq;       // [A_pk * 10]
    uint16_t motor_position; // [deg * 10]
    uint8_t is_motor_still;  // 1=still, 0=rotating

    // Packet 0x20: General Data 1
    int32_t erpm;          // [ERPM]
    int16_t duty_cycle;    // [% * 10]
    int16_t input_voltage; // [V]

    // Packet 0x21: General Data 2
    int16_t ac_current; // [A_pk * 10]
    int16_t dc_current; // [A_dc * 10]

    // Packet 0x22: General Data 3
    int16_t controller_temp; // [°C * 10]
    int16_t motor_temp;      // [°C * 10]
    uint8_t fault_code;      // 0x00=None ... 0x0A=AnalogErr

    // Packet 0x23: General Data 4
    int32_t id; // [A_pk * 100]
    int32_t iq; // [A_pk * 100]

    // Packet 0x24: General Data 5
    int8_t throttle_signal; // [%]
    int8_t brake_signal;    // [%]

    bool digital_in1;
    bool digital_in2;
    bool digital_in3;
    bool digital_in4;
    bool digital_out1;
    bool digital_out2;
    bool digital_out3;
    bool digital_out4;

    bool drive_enable;

    bool limit_cap_temp;
    bool limit_dc_current;
    bool limit_drive_enable;
    bool limit_igbt_accel_temp;
    bool limit_igbt_temp;
    bool limit_input_voltage;
    bool limit_motor_accel_temp;
    bool limit_motor_temp;
    bool limit_rpm_min;
    bool limit_rpm_max;
    bool limit_power;

    uint8_t can_map_version;

    // Packet 0x25: Configured and Available AC Currents
    int16_t max_ac_current;       // [A_pk * 10]
    int16_t avail_max_ac_current; // [A_pk * 10]
    int16_t min_ac_current;       // [A_pk * 10]
    int16_t avail_min_ac_current; // [A_pk * 10]

    // Packet 0x26: Configured and Available DC Currents
    int16_t max_dc_current;       // [A_dc * 10]
    int16_t avail_max_dc_current; // [A_dc * 10]
    int16_t min_dc_current;       // [A_dc * 10]
    int16_t avail_min_dc_current; // [A_dc * 10]

    uint64_t last_rx_time_ms;
};

class VehicleState
{
  public:
    DTI_Inverter INVERTERS[Corner::NUM_CORNERS];

    struct
    {
        struct
        {
            float channel0;
            float channel1;
            float channel2;
            float channel3;
            float channel4;
            float channel5;
            float channel6;
            float channel7;
        } Voltage;

        struct
        {
            float channel0;
            float channel1;
            float channel2;
            float channel3;
            float channel4;
            float channel5;
            float channel6;
            float channel7;
        } Raw;
    } Analog;
};
// vehicle_state.h
#pragma once
#include <array>
#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <optional>
#include <atomic>
#include <bitset>
enum class VSM_STATES {
    // Default state that is initialized to, not expected to return to unless terminal fault
    POST = 0,

    // Wait state with PREHARGE relay closed, waiting to see bus voltage rise when SC closes
    READY = 1,

    // Interim State when bus voltage rises. IF bus voltage doenst rise fast enough, terminal fault is thrown
    PRECHARGING = 2,

    // Post PRECHARGING, close AIR+, and verify that system is nominal
    // Critical safety section
    HV_ACTIVE = 3,

    // State to wait for driver to push BRAKE + START button
    ARMED = 4,

    // Interim state where buzzer is played and RTDS checks are completed
    RTDS = 5,

    // DRIVE mode, nominal with torque commands finally enabled
    DRIVE = 6,

    // FAULT mode, vehicle enters safe state with DRIVE disabled,Torque zeroed and all HV contactors opened
    FAULT = 7,

    // If HV gets disabled without AMS, IMD or BSPD signal being thrown, we enter SHUTDOWN, which then resets to READY
    // state after vehicle reaches 0 speed, 0 intermediate bus voltage. CAR must RETURN TO startup state, akin to
    // sys_reboot() AIRs CONTACTORS MUST BE RESET
    SHUTDOWN = 8,
};

enum Corner : uint8_t {
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    REAR_LEFT = 2,
    REAR_RIGHT = 3,
    NUM_CORNERS = 4
};

struct DTI_Inverter {

    static constexpr int16_t pole_pairs = 4;
    static constexpr int16_t max_ac_current_x10 = 30; // 3 A_pk — tune per motor

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

    // INPUT FROM INVERTER
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

    // TX command fields — written by control task, read by encoders
    int16_t cmd_ac_current;   // [A_pk * 10], positive = motoring, negative = regen
    uint8_t cmd_drive_enable; // 0 = disabled, 1 = enabled
};

struct Analog {
    uint16_t channels[8] = {};
    static constexpr float VREF = 5.0f;
    static constexpr float MAX_COUNT = 4096.0f;

    float get_voltage(int channel) const volatile {
        return (channels[channel] / MAX_COUNT) * VREF;
    }
};

enum APPS_ERRORS {
    PEDAL_AGREEMENT = 0,
    SHORT_CIRCUIT_P1 = 1,
    SHORT_CIRCUIT_P2 = 2,
    OPEN_CIRCUIT_P1 = 3,
    OPEN_CIRCUIT_P2 = 4,
    BRAKE_OVERLAP = 5,
    NUM_ERRORS = 8,
};

enum PEDAL_SLOPE_DIRECTION {
    POSITIVE,
    NEGATIVE
};

namespace APPS_CONSTEXPRS {
static constexpr uint16_t calculateRange(uint16_t highThreshold, uint16_t lowThreshold) {
    return (highThreshold > lowThreshold) ? (highThreshold - lowThreshold) : (lowThreshold - highThreshold);
}

static constexpr PEDAL_SLOPE_DIRECTION PEDAL_SLOPE_DIRECTION(uint16_t highThreshold, uint16_t lowThreshold) {
    return (highThreshold > lowThreshold) ? POSITIVE : NEGATIVE;
}

} // namespace APPS_CONSTEXPRS

struct APPS_data {
    bool errors[APPS_ERRORS::NUM_ERRORS];
    bool faulted;

    // ADC channel mapping
    static constexpr int pedal1_adc_channel_num = 0;
    static constexpr int pedal2_adc_channel_num = 1;
    static constexpr int brake_main_adc_channel_num = 0;

    // Drive calibration: ADC values that map to 0% (rest) and 100% (full press).
    // Used for percent calc only — must lie between the fault thresholds.
    static constexpr uint16_t pedal1_drive_rest = 2750;
    static constexpr uint16_t pedal2_drive_rest = 2750;
    static constexpr uint16_t pedal1_drive_full = 2050;
    static constexpr uint16_t pedal2_drive_full = 2050;

    // Fault thresholds: absolute ADC bounds — outside this band means a wiring fault.
    // Must bracket the drive range with margin so a normal sweep stays inside.
    static constexpr uint16_t pedal1_fault_low_adc = 1900; // open  circuit if raw < this
    static constexpr uint16_t pedal2_fault_low_adc = 1900;
    static constexpr uint16_t pedal1_fault_high_adc = 2900; // short circuit if raw > this
    static constexpr uint16_t pedal2_fault_high_adc = 2900;

    // other calibration values
    static constexpr float agreement_threshold = 0.10f; // 10% disagreement
    static constexpr int agreement_timeout_ms = 100;
    static constexpr float brake_on_threshold = 0.25f;  // throttle % to latch brake fault
    static constexpr float brake_off_threshold = 0.05f; // throttle % to clear brake fault

    // constexpr values that are calculated at compiletime
    static constexpr uint16_t pedal1_range_width =
        APPS_CONSTEXPRS::calculateRange(pedal1_drive_full, pedal1_drive_rest);
    static constexpr uint16_t pedal2_range_width =
        APPS_CONSTEXPRS::calculateRange(pedal2_drive_full, pedal2_drive_rest);
    static constexpr PEDAL_SLOPE_DIRECTION pedal1_slope_direction =
        APPS_CONSTEXPRS::PEDAL_SLOPE_DIRECTION(pedal1_drive_full, pedal1_drive_rest);
    static constexpr PEDAL_SLOPE_DIRECTION pedal2_slope_direction =
        APPS_CONSTEXPRS::PEDAL_SLOPE_DIRECTION(pedal2_drive_full, pedal2_drive_rest);

    // IN PERCENTAGE, translation of input voltages to output command AFTER APPS processing
    float commandedTorquePercentage;
    float pedal1_percent;
    float pedal2_percent;
    float average_pedal_percent;
    bool torqueVectoringEnabled = false;
};

struct BMS_data {

    static constexpr int NUM_CELLS = 72;

    struct CellData {
        uint16_t voltage;
        uint16_t open_voltage;
        uint16_t resistance;
        uint8_t balancing;
    };

    // --- 0x6B0 (8 ms) ---
    int16_t pack_current;       // × 0.1  A  (signed)
    uint16_t pack_inst_voltage; // × 0.1  V
    uint8_t pack_soc;           // × 0.5  %

    // I/O flags (byte 5)
    uint8_t multipurpose_input_2;
    uint8_t multipurpose_input_3;
    uint8_t multipurpose_output_2;
    uint8_t multipurpose_output_3;
    uint8_t multipurpose_output_4;
    uint8_t multipurpose_enable;
    uint8_t multipurpose_output;

    // Relay flags (byte 6)
    uint8_t discharge_relay;
    uint8_t charge_relay;
    uint8_t charger_safety;
    uint8_t error_mil_output;
    uint8_t multipurpose_input;
    uint8_t constant_1;
    uint8_t ready_power_signal;
    uint8_t charge_power_signal;

    // --- 0x6B1 (104 ms) ---
    uint16_t pack_dcl;       // × 1    A
    uint8_t pack_ccl;        // × 1    A
    int8_t high_temperature; // × 1    °C
    int8_t low_temperature;  // × 1    °C

    // --- 0x3E8 (104 ms) ---
    uint8_t low_cell_voltage_id;
    uint16_t low_cell_voltage;     // × 0.0001 V
    uint16_t low_opencell_voltage; // × 0.0001 V
    uint16_t low_cell_resistance;  // × 0.01   mOhm

    // --- 0x3E9 (1008 ms) ---
    uint8_t high_cell_voltage_id;
    uint16_t high_cell_voltage; // × 0.0001 V
    uint16_t high_opencell_id;
    uint16_t high_cell_resistance; // × 0.01   mOhm

    // --- 0x3F0 (96 ms) ---
    uint8_t pack_soc_ext;              // × 0.5  %
    uint16_t pack_inst_voltage_ext;    // × 0.1  V
    uint16_t pack_open_voltage = 2800; // × 0.1  V
    uint16_t pack_current_ext;         // × 0.1  A

    // --- 0x3F1 (1008 ms) ---
    uint16_t pack_resistance;  // × 0.001  Ohm
    uint16_t avg_cell_voltage; // × 0.0001 V

    // --- 0x3F2 (1000 ms) ---
    uint16_t adaptive_total_capacity; // × 0.1  Ah
    uint16_t adaptive_amphours;       // × 0.1  Ah
    uint8_t adaptive_soc;             // × 0.5  %

    // --- 0x3F3 (1000 ms) ---
    uint16_t dtc_flags_1;
    uint16_t dtc_flags_2;

    // --- 0x3F4 (1008 ms) ---
    uint16_t pack_dcl_ext; // × 1  A
    uint16_t current_limits_status;

    // --- 0x3F5 (8 ms) ---
    uint16_t pack_current_fast; // × 0.1  A

    // --- 0x200 cell broadcast (12 ms, round-robin) ---

    CellData cells[NUM_CELLS];

    int64_t last_cell_rx_time_ms;

    // General heartbeat
    int64_t last_rx_time_ms;
};

struct VSM_Data {

    static constexpr float nominal_bus_votlage = 280;
    static constexpr int RTDS_sound_length = 20;

    /**
     * @brief maximum voltage allowed across all inverters without throwing critical fault
     *
     */
    static constexpr float max_inverter_voltage_delta = 100;

    /**
     * @brief maximum time that precharging can sequence before a critical fault is thrown
     *
     */
    static constexpr float max_precharging_time = 3000;

    /**
     * @brief Calculated an estimation of the pack resistance, can be used to infer the expected voltage AT the
     * inverter terminals. This can then be used to figure out if the AIRs are opened or not even at runtime when
     * the car is not stopped and current != 0
     */
    static constexpr float estimated_pack_resistance = 0.32f;
    int64_t precharging_start_time = 0;

    int64_t RTDS_start_time = 0;

    float dc_link_voltage = 0;

    /**
     * @brief Calculated currents summed from all four inverter DC current inputs
     *
     */
    std::atomic<float> inverter_current_summed = 0;
    float estimated_link_voltage = 0;

    std::bitset<64> FAULTS;
};

// struct that provides access to sub  Interface structs that house publicly accessible data to whole program.
// classes that interact with vehicle state should refer to this struct as source of truth
class VehicleState {
  public:
    DTI_Inverter INVERTERS[Corner::NUM_CORNERS];
    Analog analogIf;
    APPS_data APPSIf;
    BMS_data BMSIf;
    const std::atomic<VSM_STATES> *VSM_STATE = nullptr;
    const VSM_Data *VSM_If = nullptr;

  private:
};

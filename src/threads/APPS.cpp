#include "threads/APPS.h"
#include "can_decoders/dti_encoders.h"
#include "vehicle_state.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(APPS, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(apps_stack, 1024);

static APPSTask apps_task_instance;

APPSTask &get_apps_task()
{
    return apps_task_instance;
}

void start_apps_task(VehicleState *v, Hardware *hw, uint32_t period_ms, int priority)
{
    apps_task_instance.set_hardware(hw);
    apps_task_instance.start(apps_stack, K_THREAD_STACK_SIZEOF(apps_stack), period_ms, priority, v, K_FP_REGS);
    LOG_INF("APPS task started (%u ms period)", period_ms);
}

void APPSTask::on_init()
{
    LOG_INF("APPS task initialized");
}

void APPSTask::on_deadline_miss()
{
    LOG_ERR("Deadline Missed in  APPS Task");
}

void APPSTask::run()
{
    APPS_data &apps = vehicle()->APPSIf;
    uint16_t &pedal1raw = vehicle()->analogIf.channels[APPS_data::pedal1_adc_channel_num];
    uint16_t &pedal2raw = vehicle()->analogIf.channels[APPS_data::pedal2_adc_channel_num];

    pedal1raw = hardware_->getADCValue(APPS_data::pedal1_adc_channel_num);
    pedal2raw = hardware_->getADCValue(APPS_data::pedal2_adc_channel_num);

    apps.errors[OPEN_CIRCUIT_P1] = checkOpenCircuit(pedal1raw, APPS_data::pedal1_low_threshold);
    apps.errors[OPEN_CIRCUIT_P2] = checkOpenCircuit(pedal2raw, APPS_data::pedal2_low_threshold);
    apps.errors[SHORT_CIRCUIT_P1] = checkShortCircuit(pedal1raw, APPS_data::pedal1_high_threshold);
    apps.errors[SHORT_CIRCUIT_P2] = checkShortCircuit(pedal2raw, APPS_data::pedal2_high_threshold);

    bool range_fault = apps.errors[OPEN_CIRCUIT_P1] || apps.errors[OPEN_CIRCUIT_P2] || apps.errors[SHORT_CIRCUIT_P1] ||
                       apps.errors[SHORT_CIRCUIT_P2];

    apps.pedal1_percent = readPedalPercent(pedal1raw, APPS_data::pedal1_low_threshold, APPS_data::pedal1_range_width,
                                           APPS_data::pedal1_slope_direction);
    apps.pedal2_percent = readPedalPercent(pedal2raw, APPS_data::pedal2_low_threshold, APPS_data::pedal2_range_width,
                                           APPS_data::pedal2_slope_direction);

    apps.errors[PEDAL_AGREEMENT] = checkPedalAgreement(apps.pedal1_percent, apps.pedal2_percent);

    float avg_pct = (apps.pedal1_percent + apps.pedal2_percent) / 2.0f;
    apps.errors[BRAKE_OVERLAP] = checkBrakeOverlap(avg_pct);

    apps.faulted = range_fault || apps.errors[PEDAL_AGREEMENT] || apps.errors[BRAKE_OVERLAP];

    apps.commandedTorquePercentage = apps.faulted ? 0.0f : avg_pct;

    if (apps.faulted)
    {
        LOG_WRN("APPS fault active — torque command zeroed");
    }

    if (apps.torqueVectoringEnabled)
    {
        // to be implemented
        LOG_ERR("TV is not implemented");
        bool &tvEnabled = const_cast<bool &>(apps.torqueVectoringEnabled);
        tvEnabled = false;
    }
    else
    {
        int16_t cmd = static_cast<int16_t>(apps.commandedTorquePercentage * vehicle()->INVERTERS[0].max_ac_current_x10);

        for (Corner c = FRONT_LEFT; c < NUM_CORNERS; c = static_cast<Corner>(1 + static_cast<int>(c)))
        {
            vehicle()->INVERTERS[c].cmd_drive_enable = apps.faulted ? 0u : 1u;
            vehicle()->INVERTERS[c].cmd_ac_current = cmd;
        }

        struct can_frame tq_command_packed_0x01{};
        encode_dti_fl_set_ac_current(&tq_command_packed_0x01, vehicle());
        hardware_->can1.send(&tq_command_packed_0x01, K_MSEC(1));
        encode_dti_fr_set_ac_current(&tq_command_packed_0x01, vehicle());
        hardware_->can1.send(&tq_command_packed_0x01, K_MSEC(1));
        encode_dti_rl_set_ac_current(&tq_command_packed_0x01, vehicle());
        hardware_->can1.send(&tq_command_packed_0x01, K_MSEC(1));
        encode_dti_rr_set_ac_current(&tq_command_packed_0x01, vehicle());
        hardware_->can1.send(&tq_command_packed_0x01, K_MSEC(1));
    }
}

float APPSTask::readPedalPercent(uint16_t raw, uint16_t low, uint16_t range, PEDAL_SLOPE_DIRECTION slope)
{
    float pct = (slope == POSITIVE) ? static_cast<float>(raw - low) / static_cast<float>(range)
                                    : static_cast<float>(low - raw) / static_cast<float>(range);

    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 1.0f)
        pct = 1.0f;
    return pct;
}

bool APPSTask::checkOpenCircuit(uint16_t raw, uint16_t low_threshold)
{
    return raw < low_threshold;
}

bool APPSTask::checkShortCircuit(uint16_t raw, uint16_t high_threshold)
{
    return raw > high_threshold;
}

bool APPSTask::checkPedalAgreement(float p1_pct, float p2_pct)
{
    if (fabsf(p1_pct - p2_pct) > APPS_data::agreement_threshold)
    {
        if (agreement_fault_deadline_ == 0)
        {
            agreement_fault_deadline_ = k_uptime_get() + APPS_data::agreement_timeout_ms;
        }
        else if (k_uptime_get() >= agreement_fault_deadline_)
        {
            LOG_WRN("Pedal agreement fault: p1=%.2f p2=%.2f", static_cast<double>(p1_pct), static_cast<double>(p2_pct));
            return true;
        }
    }
    else
    {
        agreement_fault_deadline_ = 0;
    }
    return false;
}

bool APPSTask::checkBrakeOverlap(float avg_pct)
{
    // TODO: wire up brake switch GPIO from hardware_
    bool brake_pressed = false;

    if (avg_pct > APPS_data::brake_on_threshold && brake_pressed)
    {
        brake_fault_latched_ = true;
        LOG_WRN("Brake overlap fault latched");
    }
    if (brake_fault_latched_ && avg_pct < APPS_data::brake_off_threshold)
    {
        brake_fault_latched_ = false;
        LOG_INF("Brake overlap fault cleared");
    }
    return brake_fault_latched_;
}

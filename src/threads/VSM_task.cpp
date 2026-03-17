#include "threads/VSM_task.h"
#include "can_decoders/dti_encoders.h"
#include "vehicle_state.h"
#include "zephyr/drivers/can.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/reboot.h"
#include <zephyr/sys/__assert.h>
#include <cfloat>
#include <climits>
#include <csetjmp>
#include <sys/_intsup.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(VSM, LOG_LEVEL_INF);
K_THREAD_STACK_DEFINE(VSM_stack, 2048);

static VSMTask VSM_task_instance;

void VSMTask::throw_vehicle_fault(int fault_code)
{
    STATE = VSM_STATES::FAULT;

    LOG_ERR("CRITICAL VEHICLE FAULT WAS THROWN, GOING TO FAULT STATE");
    longjmp(fault_jmp_, fault_code);
}

void VSMTask::transmit_drive_enables()
{
    for (auto c : vehicle()->INVERTERS)
    {
        c.drive_enable = DATA.drive_enabled;
    }

    struct can_frame drive_enable;
    encode_dti_fl_set_drive_enable(&drive_enable, vehicle());
    hardware_->can1.send(&drive_enable, K_MSEC(1));

    encode_dti_fr_set_drive_enable(&drive_enable, vehicle());
    hardware_->can1.send(&drive_enable, K_MSEC(1));

    encode_dti_rl_set_drive_enable(&drive_enable, vehicle());
    hardware_->can1.send(&drive_enable, K_MSEC(1));

    encode_dti_rr_set_drive_enable(&drive_enable, vehicle());
    hardware_->can1.send(&drive_enable, K_MSEC(1));
}

void VSMTask::run()
{

    int fault_code = setjmp(fault_jmp_);
    if (fault_code != 0)
    {
        STATE = VSM_STATES::FAULT;
    }
    switch (STATE)
    {
    case VSM_STATES::POST: {
        __ASSERT(system_ && hardware_ && vehicle(),
                 "System or Hardware or Vehicle struct not initialized before starting VSM. Rebooting")

        float voltage = check_inverter_voltage_skew();

        if (voltage < 5.0f)
        {
            STATE = VSM_STATES::READY;
        }
    }
    break;

    case VSM_STATES::READY: {

        // tbd : SET precharge relay High

        float voltage = check_inverter_voltage_skew();

        if (voltage > 20.0f)
        {
            DATA.precharging_start_time = k_uptime_get();
            this->STATE = VSM_STATES::PRECHARGING;
        }
    }
    break;
    case VSM_STATES::PRECHARGING: {

        if (k_uptime_get() > (DATA.precharging_start_time + DATA.max_precharging_time))
        {
            this->STATE = VSM_STATES::FAULT;
            throw_vehicle_fault(static_cast<int>(VSM_FAULTS::PRECHARGING_TOOK_TOO_LONG));
        }
        float voltage = check_inverter_voltage_skew();

        // NEEDS TO BE UPDATED TO READ LIVE BMS VOLTAGE, WILL NOT WORK AS SoC changes
        if (voltage > DATA.nominal_bus_votlage * 0.95f)
        {
            STATE = VSM_STATES::HV_ACTIVE;
        }
    }
    break;

    case VSM_STATES::HV_ACTIVE: {
        float voltage = check_inverter_voltage_skew();

        // close AIRs here
        if (voltage < DATA.nominal_bus_votlage * 0.95f)
        {
            STATE = VSM_STATES::FAULT;
            throw_vehicle_fault(static_cast<int>(VSM_FAULTS::BUS_VOLTAGE_DROPPED_AFTER_PRECHARGING));
        }

        if (voltage >= DATA.nominal_bus_votlage * 0.99f)
        {
            STATE = VSM_STATES::ARMED;
        }
    }
    break;

    case VSM_STATES::ARMED: {
        bool driver_switch;
        if (hardware_->drive_enable.get(&driver_switch) != 0)
        { // throw runtime except *\}
        }

        if (driver_switch)
        {
            STATE = VSM_STATES::RTDS;
            DATA.precharging_start_time = k_uptime_get();
        }
    }
    break;

    case VSM_STATES::RTDS: {

        if (k_uptime_get() > DATA.RTDS_start_time - (DATA.RTDS_sound_length * 3))
        {
            DATA.RTDS_start_time = k_uptime_get();
        }

        if (k_uptime_delta(&DATA.RTDS_start_time) > DATA.RTDS_sound_length)
        {
            STATE = VSM_STATES::DRIVE;
        }
    }

    break;

    case VSM_STATES::DRIVE: {
        transmit_drive_enables();

        // calculate in local variable to be loaded in atomically later.
        int local_current_calculated = 0;
        for (auto c : vehicle()->INVERTERS)
        {
            local_current_calculated += c.dc_current;
        }

        // atomic operation for irq protection
        DATA.inverter_current_summed.store(local_current_calculated);
    }
    break;

    case VSM_STATES::SHUTDOWN:
        break;

    case VSM_STATES::FAULT: {
        static int fault_code_latched = -1;

        DATA.drive_enabled = 0;
        transmit_drive_enables();
        if (fault_code != 0)
        {
            fault_code_latched = fault_code;
        }

        // SAFE AIR CTRL
        // zero out tq commands

        LOG_ERR("IN FAULTED STATE WITH CODE: %d", fault_code_latched);
    }
    break;

        __ASSERT(false, "UNREACHABLE STATEMENT IN VSM ");
    }

    return;
}

VSMTask &get_VSM_task()
{
    return VSM_task_instance;
}
void VSMTask::injectVehicleState(void)
{
    this->vehicle()->VSM_STATE = &this->STATE;
    this->vehicle()->VSM_If = &this->DATA;
}

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms, int priority)
{
    VSM_task_instance.set_system(sys);
    VSM_task_instance.set_hardware(hw);
    VSM_task_instance.start(VSM_stack, K_THREAD_STACK_SIZEOF(VSM_stack), period_ms, priority, v, K_FP_REGS);
    VSM_task_instance.injectVehicleState();
    LOG_INF("VSM task started (%u ms period)", period_ms);
}

[[nodiscard]] float VSMTask::check_inverter_voltage_skew()
{

    float averageVoltage = 0;
    float minVoltage = FLT_MAX;
    float maxVoltage = FLT_MIN;

    int key = irq_lock();

    for (int i = 0; i < 4; i++)
    {
        int curVoltage = vehicle()->INVERTERS[i].input_voltage;
        averageVoltage += curVoltage / 4.0f;

        if (curVoltage < minVoltage)
        {
            minVoltage = curVoltage;
        }
        else if (curVoltage > maxVoltage)
        {
            maxVoltage = curVoltage;
        }
    }

    irq_unlock(key);

    [[unlikely]] if (fabsf(maxVoltage - minVoltage) > DATA.max_inverter_voltage_delta)
    {
        LOG_ERR("Voltage SKEW TOO GREAT across Inverters");
        STATE = VSM_STATES::FAULT;
        throw_vehicle_fault(120);
    }
    return averageVoltage;
}
#include "threads/VSM_task.h"
#include "can_decoders/dti_encoders.h"
#include "vehicle_state.h"
#include "zephyr/drivers/can.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/reboot.h"
#include <bitset>
#include <cstdint>
#include <tuple>
#include <utility>
#include <zephyr/sys/__assert.h>
#include <cfloat>
#include <climits>
#include <exception>
#include <csetjmp>
#include <sys/_intsup.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(VSM, LOG_LEVEL_INF);
K_THREAD_STACK_DEFINE(VSM_stack, 2048);

static VSMTask VSM_task_instance;

void VSMTask::transmit_drive_enables()
{

    if (STATE == VSM_STATES::READY)
    {
        for (auto c : vehicle()->INVERTERS)
        {
            c.cmd_drive_enable = 1;
        }
    }
    else
    {
        for (auto c : vehicle()->INVERTERS)
        {
            c.cmd_drive_enable = 0;
        }
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

    hardware_->led_orange.toggle();

    VSM_FAULTS fault_state = VSM_FAULTS::NO_FAULT;

    switch (this->STATE)
    {
    case VSM_STATES::POST: {
        fault_state = run_post();
    }
    break;
    case VSM_STATES::READY: {
        fault_state = run_ready();
    }
    break;

    case VSM_STATES::PRECHARGING: {
        fault_state = run_ready();
    }
    break;

    case VSM_STATES::HV_ACTIVE: {
        fault_state = run_hv_active();
    }
    break;

    case VSM_STATES::ARMED: {
        fault_state = run_armed();
    }
    break;

    case VSM_STATES::RTDS: {
        fault_state = run_rtds();
    }
    break;

    case VSM_STATES::DRIVE: {
        fault_state = run_drive();
    }
    break;

    case VSM_STATES::FAULT: {
        fault_state = run_fault();
    }
    break;

    case VSM_STATES::SHUTDOWN: {
        fault_state = run_shutdown();
    }
    break;

    default:

        __ASSERT(false, "FALL THROUGH IN VSM SWITCH CASE");
    }

    if (fault_state != VSM_FAULTS::NO_FAULT)
    {
        DATA.FAULTS.set(std::to_underlying(fault_state));
        STATE = VSM_STATES::FAULT;
        std::ignore = run_fault();
        LOG_ERR("CRITICAL VEHICLE FAULT WAS THROWN, GOING TO FAULT STATE, Code: %d ", std::to_underlying(fault_state));
        LOG_ERR("VEHICLE FAULT VECTOR: %llu", DATA.FAULTS.to_ullong());

        return;
    }
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

void VSMTask::check_faults(void)
{
    // checking for inverter voltage skew
    InvertersAggregate<int16_t> inp_voltage = reduce_inverter(&DTI_Inverter::input_voltage);
    [[unlikely]] if (inp_voltage.skew() > DATA.max_inverter_voltage_delta)
    {
        DATA.FAULTS.set(std::to_underlying(VSM_FAULTS::INVERTER_VOLTAGE_SKEW), true);
    }

    // ADD GPIO checks for shutdown faults
}

VSM_FAULTS VSMTask::run_post()
{
    __ASSERT(system_ && hardware_ && vehicle(),
             "System or Hardware or Vehicle struct not initialized before starting VSM. Rebooting");

    check_faults();
    float voltage = reduce_inverter(&DTI_Inverter::input_voltage).avg();

    if (voltage < 5.0f)
    {
        STATE = VSM_STATES::READY;
    }

    return VSM_FAULTS::NO_FAULT;
}

VSM_FAULTS VSMTask::run_ready()
{
    // checks if any faults were set in the check_fault bitset, if set call FAULT handlers
    check_faults();
    if (DATA.FAULTS.any())
    {
        return VSM_FAULTS::FAULTED;
    }
    // tbd : SET precharge relay High

    float voltage = reduce_inverter(&DTI_Inverter::input_voltage).avg();

    if (voltage > 20.0f)
    {
        DATA.precharging_start_time = k_uptime_get();
        this->STATE = VSM_STATES::PRECHARGING;
    }

    return VSM_FAULTS::NO_FAULT;
}

VSM_FAULTS VSMTask::run_precharging()
{
    if (k_uptime_get() > (DATA.precharging_start_time + DATA.max_precharging_time))
    {
        this->STATE = VSM_STATES::FAULT;
        return VSM_FAULTS::PRECHARGING_TOOK_TOO_LONG;
    }
    float voltage = reduce_inverter(&DTI_Inverter::input_voltage).avg();

    // NEEDS TO BE UPDATED TO READ LIVE BMS VOLTAGE, WILL NOT WORK AS SoC changes
    if (voltage > DATA.nominal_bus_votlage * 0.95f)
    {
        STATE = VSM_STATES::HV_ACTIVE;
    }

    return VSM_FAULTS::NO_FAULT;
}
VSM_FAULTS VSMTask::run_hv_active()
{
    float voltage = reduce_inverter(&DTI_Inverter::input_voltage).avg();

    // close AIRs here
    if (voltage < DATA.nominal_bus_votlage * 0.95f)
    {
        STATE = VSM_STATES::FAULT;
        return VSM_FAULTS::BUS_VOLTAGE_DROPPED_AFTER_PRECHARGING;
    }

    if (voltage >= DATA.nominal_bus_votlage * 0.99f)
    {
        STATE = VSM_STATES::ARMED;
    }

    return VSM_FAULTS::NO_FAULT;
}
VSM_FAULTS VSMTask::run_armed()
{
    bool driver_switch;
    if (hardware_->drive_enable.get(&driver_switch) != 0)
    { // throw runtime except *\}
    }

    if (driver_switch)
    {
        STATE = VSM_STATES::RTDS;
        DATA.precharging_start_time = k_uptime_get();
    }

    return VSM_FAULTS::NO_FAULT;
}
VSM_FAULTS VSMTask::run_rtds()
{
    if (k_uptime_get() > DATA.RTDS_start_time - (DATA.RTDS_sound_length * 3))
    {
        DATA.RTDS_start_time = k_uptime_get();
    }

    if (k_uptime_delta(&DATA.RTDS_start_time) > DATA.RTDS_sound_length)
    {
        STATE = VSM_STATES::DRIVE;
    }

    return VSM_FAULTS::NO_FAULT;
}
VSM_FAULTS VSMTask::run_drive()
{
    transmit_drive_enables();

    InvertersAggregate<int16_t> inv_dc_current = reduce_inverter(&DTI_Inverter::dc_current);
    DATA.inverter_current_summed.store(inv_dc_current.sum / 10.0f);

    InvertersAggregate<int16_t> inv_dc_voltage = reduce_inverter(&DTI_Inverter::input_voltage);
    DATA.dc_link_voltage = inv_dc_voltage.avg();
    DATA.estimated_link_voltage = vehicle()->VSM_If->nominal_bus_votlage * DATA.estimated_pack_resistance;

    if (DATA.dc_link_voltage < DATA.estimated_link_voltage)
    {
        LOG_WRN("AIRs Opened");
        // OPEN AIRs, HARD RESET TO POST STATE.
        STATE = VSM_STATES::SHUTDOWN;
    }

    return VSM_FAULTS::NO_FAULT;
}
VSM_FAULTS VSMTask::run_fault()
{
    static int fault_code_latched = -1;

    // SAFE AIR CTRL
    // zero out tq commands    transmit_drive_enables();

    LOG_ERR("IN FAULTED STATE WITH CODE: %d", fault_code_latched);

    return VSM_FAULTS::NO_FAULT;
}

VSM_FAULTS VSMTask::run_shutdown()
{
}

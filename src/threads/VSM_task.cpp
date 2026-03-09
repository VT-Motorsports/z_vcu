#include "threads/VSM_task.h"
#include "vehicle_state.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/reboot.h"
#include <cfloat>
#include <climits>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

LOG_MODULE_REGISTER(VSM, LOG_LEVEL_INF);
K_THREAD_STACK_DEFINE(VSM_stack, 2048);

static VSMTask VSM_task_instance;

void VSMTask::run()
{
    switch (STATE)
    {
    case VSM_STATES::POST: {

        if (!system_ || !hardware_ || !vehicle())
        {
            LOG_ERR("System or Hardware or Vehicle struct not initialized before starting VSM. Rebooting");
            sys_reboot(SYS_REBOOT_COLD);
        }
        else
        {
            float voltage = check_inverter_voltage_skew();

            if (voltage < 5.0f)
            {
                STATE = VSM_STATES::READY;
            }
        }
    }
    break;

    case VSM_STATES::READY: {

        // tbd : SET precharge relay High

        float voltage = check_inverter_voltage_skew();

        if (voltage > 20.0f)
        {
            precharging_start_time = k_uptime_get();
            this->STATE = VSM_STATES::PRECHARGING;
        }
    }
    break;
    case VSM_STATES::PRECHARGING: {

        if (k_uptime_get() > (precharging_start_time + vehicle()->VSM_If.max_precharging_time))
        {
            this->STATE = VSM_STATES::FAULT;
        }
        float voltage = check_inverter_voltage_skew();

        // NEEDS TO BE UPDATED TO READ LIVE BMS VOLTAGE, WILL NOT WORK AS SoC changes
        if (voltage > vehicle()->VSM_If.nominal_bus_votlage * 0.95f)
        {
            STATE = VSM_STATES::HV_ACTIVE;
        }
    }
    break;

    case VSM_STATES::HV_ACTIVE: {
        float voltage = check_inverter_voltage_skew();

        if (voltage < vehicle()->VSM_If.nominal_bus_votlage * 0.95f)
        {
            STATE = VSM_STATES::FAULT;
            LOG_ERR("BUS VOLTAGE DROPPED AFTER PRECHARGING");
        }

        if ()
    }
    break;

    case VSM_STATES::ARMED:
        break;

    case VSM_STATES::RTDS:
        break;

    case VSM_STATES::DRIVE:
        break;

    case VSM_STATES::SHUTDOWN:
        break;

    case VSM_STATES::FAULT:

        break;
    }
}

VSMTask &get_VSM_task()
{
    return VSM_task_instance;
}
void VSMTask::injectVehicleState(void)
{
    this->vehicle()->VSM_STATE = &this->STATE;
}

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms, int priority)
{
    VSM_task_instance.set_system(sys);
    VSM_task_instance.set_hardware(hw);
    VSM_task_instance.start(VSM_stack, K_THREAD_STACK_SIZEOF(VSM_stack), period_ms, priority, v, K_FP_REGS);
    VSM_task_instance.injectVehicleState();
    LOG_INF("VSM task started (%u ms period)", period_ms);
}

float VSMTask::check_inverter_voltage_skew()
{

    float averageVoltage = 0;
    float minVoltage = FLT_MAX;
    float maxVoltage = FLT_MIN;

    int key = irq_lock();

    for (int i = 0; i < 4; i++)
    {
        int curVoltage = vehicle()->INVERTERS[i].input_voltage;
        averageVoltage += curVoltage / 4.0;

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

    [[unlikely]] if (fabsf(maxVoltage - minVoltage) > vehicle()->VSM_If.max_inverter_voltage_delta)
    {
        LOG_ERR("Voltage SKEW TOO GREAT across Inverters");
        STATE = VSM_STATES::FAULT;
    }
    return averageVoltage;
}
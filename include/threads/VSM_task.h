#pragma once

#include "vehicle_state.h"
#include "hardware.h"
#include "threads/periodic_task.h"
#include "threads/system.h"
#include <atomic>

class VSMTask : public PeriodicTask<VSMTask>
{
    friend class PeriodicTask<VSMTask>;

  public:
    void set_system(System *sys)
    {
        system_ = sys;
    }
    void set_hardware(Hardware *hw)
    {
        hardware_ = hw;
    }
    void injectVehicleState(void);

  private:
    System *system_ = nullptr;
    Hardware *hardware_ = nullptr;
    std::atomic<VSM_STATES> STATE = VSM_STATES::POST;

    /**
     * @brief checks voltage across all 4 inverters
     * @attention MAY throw critical fault over CAN/LOGS and move VSM to FAULT state
                 if the voltage delta across all four invertes exceeds this-> max_inverter_voltage_delta
     * @return float returns average voltage across 4 inverters
     */

    [[nodiscard]] float check_inverter_voltage_skew();

    /**
     * @brief maximum voltage allowed across all inverters without throwing critical fault
     *
     */

    /**
     * @brief maximum time that precharging can sequence before a critical fault is thrown
     *
     */
    int64_t precharging_start_time = 0.0f;
    void run();
};

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms = 50, int priority = -5);

VSMTask &get_VSM_task();
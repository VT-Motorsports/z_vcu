#pragma once

#include "vehicle_state.h"
#include "hardware.h"
#include "threads/periodic_task.h"
#include "threads/system.h"
#include <atomic>
#include <csetjmp>

enum class VSM_FAULTS : int
{
    /**
     * @brief FAULT CODE THROWN WHEN VOLTAGE IS NOT CONSISTENT ACROSS ALL FOUR INVERTERS
     *
     */
    INVERTER_VOLTAGE_SKEW = 120,
    PRECHARGING_TOOK_TOO_LONG = 121,
    BUS_VOLTAGE_DROPPED_AFTER_PRECHARGING = 122,
};

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
    jmp_buf fault_jmp_;

    VSM_Data DATA;

    void throw_vehicle_fault(int fault_code);

    /**
     * @brief checks voltage across all 4 inverters
     * @attention MAY throw critical fault over CAN/LOGS and move VSM to FAULT state
                 if the voltage delta across all four invertes exceeds this-> max_inverter_voltage_delta
     * @return float returns average voltage across 4 inverters
     */

    [[nodiscard]] float check_inverter_voltage_skew();

    void transmit_drive_enables();

    void run();
};

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms = 50, int priority = -5);

VSMTask &get_VSM_task();
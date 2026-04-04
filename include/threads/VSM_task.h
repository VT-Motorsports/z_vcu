#pragma once

#include "vehicle_state.h"
#include "hardware.h"
#include "threads/periodic_task.h"
#include "threads/system.h"
#include <atomic>
#include <bitset>
#include <csetjmp>

enum class VSM_FAULTS : int
{
    NO_FAULT = 0,
    FAULTED = 4, // used when fault type is unknown but we know that a fault has occured
    INVERTER_VOLTAGE_SKEW = 12,
    PRECHARGING_TOOK_TOO_LONG = 13,
    BUS_VOLTAGE_DROPPED_AFTER_PRECHARGING = 14,
    IMD_FAULT = 1,
    AMS_FAULT = 2,
    BSPD_FAULT = 3,
};

template <typename T> struct InvertersAggregate
{
    T sum;
    T min;
    T max;
    T avg() const
    {
        return sum / static_cast<T>(4);
    }
    T skew() const
    {
        return max - min;
    }
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

    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_post();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_ready();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_precharging();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_hv_active();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_armed();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_rtds();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_drive();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_fault();
    [[nodiscard("Do not discard fault return on VSM function")]] VSM_FAULTS run_shutdown();

    VSM_Data DATA;

    /**
     * @brief Does not check for all faults, only checks for common STATE agnostic faults
     *  such as shutdown faults, skew faults or bus voltage/current faults
     *  STATE concisous faults such as precharging or RTDS should be checked in teh VSM itself
     *  @return modifies DATA.FAULTS bitvector with recognized faults
     */
    void check_faults(void);

    void transmit_drive_enables();

    void run();

    template <typename T> InvertersAggregate<T> const reduce_inverter(T DTI_Inverter::*field) const
    {
        T sum = 0, mn = vehicle()->INVERTERS[0].*field, mx = mn;
        for (int i = 0; i < 4; i++)
        {
            T val = vehicle()->INVERTERS[i].*field;
            sum += val;
            if (val < mn)
                mn = val;
            if (val > mx)
                mx = val;
        }
        return {sum, mn, mx};
    }
};

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms = 50, int priority = -5);

VSMTask &get_VSM_task();
#pragma once

#include "vehicle_state.h"
#include "hardware.h"
#include "threads/periodic_task.h"
#include "threads/system.h"


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

  private:
    System *system_ = nullptr;
    Hardware *hardware_ = nullptr;
    VSM_STATES STATE = VSM_STATES::POST;

    void run();
};

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms = 50, int priority = 2);

VSMTask &get_VSM_task();
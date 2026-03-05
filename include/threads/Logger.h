#pragma once

#include "hardware.h"
#include "threads/periodic_task.h"
#include "threads/system.h"
#include "vehicle_state.h"

class LoggerTask : public PeriodicTask<LoggerTask>
{
    friend class PeriodicTask<LoggerTask>;

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

    void run();
};

void start_logger_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms = 50, int priority = 2);

LoggerTask &get_logger_task();
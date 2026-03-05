#pragma once

#include "hardware.h"
#include "threads/periodic_task.h"
#include "vehicle_state.h"

class APPSTask : public PeriodicTask<APPSTask>
{
    friend class PeriodicTask<APPSTask>;

  public:
    void set_hardware(Hardware *hw)
    {
        hardware_ = hw;
    }

  private:
    Hardware *hardware_ = nullptr;
    int64_t agreement_fault_deadline_ = 0;
    bool brake_fault_latched_ = false;

    void on_init();
    void on_deadline_miss();
    void run();

    float readPedalPercent(uint16_t raw, uint16_t low, uint16_t range, PEDAL_SLOPE_DIRECTION slope);
    bool checkOpenCircuit(uint16_t raw, uint16_t low_threshold);
    bool checkShortCircuit(uint16_t raw, uint16_t high_threshold);
    bool checkPedalAgreement(float p1_pct, float p2_pct);
    bool checkBrakeOverlap(float avg_pct);
};

// Start the APPS task. Stack and task instance are owned inside APPS.cpp.
void start_apps_task(VehicleState *v, Hardware *hw, uint32_t period_ms = 10, int priority = 5);

APPSTask &get_apps_task();

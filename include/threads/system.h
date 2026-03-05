#pragma once

#include "hardware.h"
#include "threads/periodic_task.h"
#include <zephyr/debug/cpu_load.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/mem_stats.h>
#include <zephyr/sys/sys_heap.h>

// System manages the shared heap and exposes system-level stats.
// Threading has moved to DiagnosticsTask below.
class System
{
public:
    int init();

    void    *heap_alloc(size_t size, k_timeout_t timeout);
    void     heap_free(void *ptr);
    int      get_heap_stats(struct sys_memory_stats *stats);
    uint64_t get_uptime_ms() const;
    uint8_t  get_cpu_load() const;
    bool     is_initialized() const { return initialized_; }

private:
    struct k_heap heap_;
    bool          initialized_ = false;
};

// Periodic diagnostics: logs uptime, heap, CPU load, and CAN bus states.
class DiagnosticsTask : public PeriodicTask<DiagnosticsTask>
{
    friend class PeriodicTask<DiagnosticsTask>;

public:
    void set_system(System *sys)    { system_   = sys; }
    void set_hardware(Hardware *hw) { hardware_ = hw;  }

private:
    System   *system_   = nullptr;
    Hardware *hardware_ = nullptr;

    void run();
};

// Start the diagnostics task. Stack and instance are owned inside system.cpp.
void start_diagnostics_task(System *sys, Hardware *hw, VehicleState *v,
                             uint32_t period_ms = 1000, int priority = 10);

DiagnosticsTask &get_diagnostics_task();

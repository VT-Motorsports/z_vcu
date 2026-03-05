#pragma once

#include "vehicle_state.h"
#include <zephyr/kernel.h>

// CRTP base for all periodic VCU tasks.
//
// Usage:
//   class MyTask : public PeriodicTask<MyTask> {
//       friend class PeriodicTask<MyTask>;
//   private:
//       void on_init()          { /* one-time setup */ }          // optional
//       void run()              { /* periodic work  */ }          // required
//       void on_deadline_miss() { set_period(get_period() * 2); } // optional
//   };
//
//   K_THREAD_STACK_DEFINE(my_stack, 1024);
//   static MyTask my_task;
//
//   // In main() after hardware init:
//   my_task.start(my_stack, K_THREAD_STACK_SIZEOF(my_stack),
//                 /*period_ms=*/1, /*priority=*/-5, &vehicle);
//
// Timing: each thread sleeps to an absolute wake target incremented by period_ms
// every iteration — k_sleep(K_TIMEOUT_ABS_MS(next_wake)). Execution time of run()
// is absorbed into the sleep interval rather than accumulating as drift.
//
// Deadline detection: if k_uptime_get() > next_wake before sleeping, the previous
// run() overran. on_deadline_miss() is called and k_sleep returns immediately
// (target already in the past). next_wake is still advanced normally, so the thread
// self-corrects after a single overrun without backlog accumulation.
//
// Runtime rate adjustment: set_period(ms) takes effect on the next loop iteration.
// Calling it from on_deadline_miss() is the standard self-degradation pattern.

template <typename Derived> class PeriodicTask
{
  public:
    // Start the task thread.
    //   stack / stack_size : from K_THREAD_STACK_DEFINE — cannot be a class member.
    //   period_ms          : initial period in milliseconds.
    //   priority           : Zephyr thread priority.
    //                        Negative = cooperative (non-preemptible by other threads).
    //                        Positive = preemptible.
    //   vehicle            : shared vehicle state, accessible via vehicle() in derived.
    //   flags              : k_thread_create flags, e.g. K_FP_REGS for FP-using tasks.
    void start(k_thread_stack_t *stack, size_t stack_size, uint32_t period_ms, int priority, VehicleState *vehicle,
               uint32_t flags = 0);

    // Change the task period at runtime. Takes effect on the next loop iteration.
    void set_period(uint32_t ms)
    {
        period_ms_ = ms;
    }

    uint32_t get_period() const
    {
        return period_ms_;
    }
    uint32_t get_misses() const
    {
        return deadline_misses_;
    }
    uint32_t get_total_runs() const
    {
        return total_runs_;
    }
    bool is_running() const
    {
        return running_;
    }

  protected:
    VehicleState *vehicle()
    {
        return vehicle_;
    }

    // Default no-op hooks. Derived classes shadow these to override.
    void on_init()
    {
    }
    void on_deadline_miss()
    {
    }

  private:
    struct k_thread thread_;
    volatile uint32_t period_ms_ = 0;
    VehicleState *vehicle_ = nullptr;
    volatile bool running_ = false;
    uint32_t deadline_misses_ = 0;
    uint32_t total_runs_ = 0;

    static void entry(void *self, void *, void *);
};

// --- Template implementation (must live in header) ---

template <typename Derived>
void PeriodicTask<Derived>::start(k_thread_stack_t *stack, size_t stack_size, uint32_t period_ms, int priority,
                                  VehicleState *vehicle, uint32_t flags)
{
    period_ms_ = period_ms;
    vehicle_ = vehicle;
    running_ = true;
    k_thread_create(&thread_, stack, stack_size, entry, this, nullptr, nullptr, priority, flags, K_NO_WAIT);
}

template <typename Derived> void PeriodicTask<Derived>::entry(void *self, void *, void *)
{
    auto *task = static_cast<Derived *>(self);

    task->on_init();

    int64_t next_wake = k_uptime_get();

    while (task->running_)
    {
        next_wake += static_cast<int64_t>(task->period_ms_);

        // Detect overrun: previous run() took longer than period_ms.
        if (k_uptime_get() > next_wake)
        {
            task->deadline_misses_++;
            task->on_deadline_miss();
            // k_sleep below returns immediately when target is in the past.
        }

        k_sleep(K_TIMEOUT_ABS_MS(next_wake));
        task->run();
        task->total_runs_++;
    }
}

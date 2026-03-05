#include "threads/system.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(system, LOG_LEVEL_INF);

K_HEAP_DEFINE(system_heap, 2048);
K_THREAD_STACK_DEFINE(diag_stack, 2048);

static DiagnosticsTask diagnostics_task_instance;

DiagnosticsTask &get_diagnostics_task()
{
    return diagnostics_task_instance;
}

void start_diagnostics_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms, int priority)
{
    diagnostics_task_instance.set_system(sys);
    diagnostics_task_instance.set_hardware(hw);
    diagnostics_task_instance.start(diag_stack, K_THREAD_STACK_SIZEOF(diag_stack), period_ms, priority, v);
    LOG_INF("Diagnostics task started (%u ms period)", period_ms);
}

// --- System ---

int System::init()
{
    LOG_INF("Initializing system resources...");

    void *test_ptr = k_heap_alloc(&system_heap, 64, K_NO_WAIT);
    if (!test_ptr)
    {
        LOG_ERR("Heap initialization failed");
        return -1;
    }
    k_heap_free(&system_heap, test_ptr);

    heap_ = system_heap;
    initialized_ = true;

    LOG_INF("System initialized");
    return 0;
}

void *System::heap_alloc(size_t size, k_timeout_t timeout)
{
    if (!initialized_)
        return nullptr;
    return k_heap_alloc(&heap_, size, timeout);
}

void System::heap_free(void *ptr)
{
    if (!initialized_ || !ptr)
        return;
    k_heap_free(&heap_, ptr);
}

int System::get_heap_stats(struct sys_memory_stats *stats)
{
    if (!initialized_ || !stats)
        return -1;
    return sys_heap_runtime_stats_get(&heap_.heap, stats);
}

uint64_t System::get_uptime_ms() const
{
    return k_uptime_get();
}

uint8_t System::get_cpu_load() const
{
    return cpu_load_get(1);
}

// --- DiagnosticsTask ---

void DiagnosticsTask::run()
{
    uint64_t uptime_ms = k_uptime_get();

    struct sys_memory_stats mem_stats = {0, 0, 0};
    system_->get_heap_stats(&mem_stats);

    uint8_t cpu_load = system_->get_cpu_load();

    LOG_INF("Uptime: %llu ms | Heap: %zu/%zu bytes | CPU: %d.%d%%", uptime_ms, mem_stats.allocated_bytes,
            mem_stats.allocated_bytes + mem_stats.free_bytes, cpu_load / 10, cpu_load % 10);

    if (hardware_ && hardware_->can1.is_initialized())
    {
        enum can_state state;
        if (hardware_->can1.get_state(&state) == 0)
        {
            switch (state)
            {
            case CAN_STATE_ERROR_ACTIVE:
                LOG_INF("CAN1: Active");
                break;
            case CAN_STATE_ERROR_WARNING:
                LOG_WRN("CAN1: Warning");
                break;
            case CAN_STATE_ERROR_PASSIVE:
                LOG_WRN("CAN1: Error Passive");
                break;
            case CAN_STATE_BUS_OFF:
                LOG_ERR("CAN1: Bus Off");
                break;
            case CAN_STATE_STOPPED:
                LOG_INF("CAN1: Stopped");
                break;
            default:
                LOG_ERR("CAN1: Unknown state");
                break;
            }
        }
    }

    if (hardware_ && hardware_->can2.is_initialized())
    {
        enum can_state state;
        if (hardware_->can2.get_state(&state) == 0)
        {
            switch (state)
            {
            case CAN_STATE_ERROR_ACTIVE:
                LOG_INF("CAN2: Active");
                break;
            case CAN_STATE_ERROR_WARNING:
                LOG_WRN("CAN2: Warning");
                break;
            case CAN_STATE_ERROR_PASSIVE:
                LOG_WRN("CAN2: Error Passive");
                break;
            case CAN_STATE_BUS_OFF:
                LOG_ERR("CAN2: Bus Off");
                break;
            case CAN_STATE_STOPPED:
                LOG_INF("CAN2: Stopped");
                break;
            default:
                LOG_ERR("CAN2: Unknown state");
                break;
            }
        }
    }
}

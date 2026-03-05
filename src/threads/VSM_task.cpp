#include "threads/VSM_task.h"
#include "vehicle_state.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(VSM, LOG_LEVEL_INF);
K_THREAD_STACK_DEFINE(VSM_stack, 2048);

static VSMTask VSM_task_instance;

void VSMTask::run()
{
    this->vehicle()->VSM_STATE = &STATE;
}

VSMTask &get_VSM_task()
{
    return VSM_task_instance;
}

void start_VSM_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms, int priority)
{

    VSM_task_instance.set_system(sys);
    VSM_task_instance.set_hardware(hw);
    VSM_task_instance.start(VSM_stack, K_THREAD_STACK_SIZEOF(VSM_stack), period_ms, priority, v, K_FP_REGS);
    LOG_INF("VSM task started (%u ms period)", period_ms);
}
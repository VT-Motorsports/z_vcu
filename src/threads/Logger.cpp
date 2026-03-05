#include "threads/Logger.h"
#include "can_decoders/logger_encoders.h"
#include "vehicle_state.h"
#include "zephyr/kernel.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(logger, LOG_LEVEL_INF);
K_THREAD_STACK_DEFINE(logger_stack, 2048);

static LoggerTask logger_task_instance;

void LoggerTask::run()
{
    struct can_frame frame{};
    encode_apps_state(&frame, vehicle());
    hardware_->can2.send(&frame, K_NO_WAIT);
}

LoggerTask &get_logger_task()
{
    return logger_task_instance;
}

void start_logger_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms, int priority)
{
    logger_task_instance.set_system(sys);
    logger_task_instance.set_hardware(hw);
    logger_task_instance.start(logger_stack, K_THREAD_STACK_SIZEOF(logger_stack), period_ms, priority, v, K_FP_REGS);
    LOG_INF("Logger task started (%u ms period)", period_ms);
}
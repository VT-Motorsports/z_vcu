#include "threads/Logger.h"
#include "can_decoders/logger_encoders.h"
#include "vehicle_state.h"
#include "zephyr/kernel.h"
#include <cstdint>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(logger, LOG_LEVEL_INF);
K_THREAD_STACK_DEFINE(logger_stack, 2048);

static LoggerTask logger_task_instance;

void LoggerTask::run() {
    hardware_->led_blue.toggle();
    struct can_frame frame{};
    encode_apps_state(&frame, vehicle());
    hardware_->can1.send(&frame, K_MSEC(1));
    for (int i = 0; i < 8; i++) {
        vehicle()->analogIf.channels[i] = hardware_->getADCValue(i);
    }

    // work to encode and send analog frames (8 channels, 4 per frame = 2 frames)
    struct can_frame analog_frames[2];
    encode_analog_channels(analog_frames, vehicle());
    for (const can_frame analog_frame : analog_frames) {
        hardware_->can1.send(&analog_frame, K_MSEC(1));
    }

    // work to encode and send VSM frames
}

LoggerTask &get_logger_task() {
    return logger_task_instance;
}

void start_logger_task(System *sys, Hardware *hw, VehicleState *v, uint32_t period_ms, int priority) {
    logger_task_instance.set_system(sys);
    logger_task_instance.set_hardware(hw);
    logger_task_instance.start(logger_stack, K_THREAD_STACK_SIZEOF(logger_stack), period_ms, priority, v, K_FP_REGS);
    LOG_INF("Logger task started (%u ms period)", period_ms);
}
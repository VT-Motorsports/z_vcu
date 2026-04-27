#include "AIR.h"
#include "threads/VSM_task.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/__assert.h"

#include <tuple>
#include <utility>

LOG_MODULE_REGISTER(contactor);

contactor::contactor() : gpioa_(DEVICE_DT_GET(DT_NODELABEL(gpioa))) {
}

int contactor::init() {
    // hardcoded for AIR+ ctrl pin on VCU v2 2026
    int code = gpio_ref.init(gpioa_, 8, GPIO_OUTPUT_LOW);

    return code;
}

VSM_FAULTS contactor::arm() {

    if (is_faulted) {
        LOG_ERR("Attempt to arm Contactor while Faulted");
        return VSM_FAULTS::CONTACTOR_ATTEMPT_ARM_WHILE_FAULTED;
    }

    LOG_WRN("Contactor Armed");
    is_armed = true;
    return VSM_FAULTS::NO_FAULT;
}

VSM_FAULTS contactor::disarm() {

    if (is_closed) {
        gpio_ref.set(false);
        is_closed = false;
        LOG_ERR("Contactor Disarmed before Opening. Contactor still opened and disarmed");
    }

    is_armed = false;
    return VSM_FAULTS::NO_FAULT;
}

int contactor::open() {
    is_armed = false;
    gpio_ref.set(false);
    LOG_INF("Contactor opened");
    is_closed = false;

    return 0;
}

bool contactor::get_armed() const {
    return is_armed;
}

bool contactor::get_closed() const {
    return is_closed;
}

VSM_FAULTS contactor::close() {

    if (!is_armed) {
        LOG_ERR("Attempted to close contactor before arming. Not Allowed");
        std::ignore = this->throw_fault();
        return (VSM_FAULTS::CONTACTOR_ATTEMPT_CLOSE_BEFORE_ARM);
    }
    if (is_faulted) {
        LOG_ERR("Contactor is Faulted, Cannot close Contactor after fault");
        return VSM_FAULTS::CONTACTOR_FAULTED;
    }
    gpio_ref.set(true);
    LOG_INF("Contactor Closed ");
    is_armed = false;
    is_closed = true;
    return VSM_FAULTS::NO_FAULT;
}

VSM_FAULTS contactor::throw_fault() {
    LOG_ERR("Contactor has been placed into fault state. Cannot be cleared without restarting vehicle");
    std::ignore = this->open();
    is_faulted = true;

    return VSM_FAULTS::CONTACTOR_FAULTED;
}

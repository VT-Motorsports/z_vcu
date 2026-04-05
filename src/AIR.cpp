#include "AIR.h"
#include "threads/VSM_task.h"
#include "zephyr/drivers/gpio.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/__assert.h"

#include <tuple>

LOG_MODULE_REGISTER(contactor);

contactor::contactor() : gpioa_(DEVICE_DT_GET(DT_NODELABEL(gpioa)))
{
}

int contactor::init()
{
    // hardcoded for AIR+ ctrl pin on VCU v2 2026
    int code = gpio_ref.init(gpioa_, 8, GPIO_OUTPUT_LOW);

    return code;
}

int contactor::arm()
{

    if (is_faulted)
    {
        LOG_ERR("Attempt to arm Contactor while Faulted");
        // swap to enum type
        return -1;
    }

    LOG_WRN("Contactor Armed");
    is_armed = true;
    return 0;
}

int contactor::disarm()
{

    if (is_closed)
    {
        gpio_ref.set(false);
        is_closed = false;
        LOG_ERR("Contactor Disarmed before Opening. Contactor still opened and disarmed");
    }

    is_armed = false;
    return 0;
}

int contactor::open()
{
    is_armed = false;
    gpio_ref.set(false);
    LOG_INF("Contactor opened");
    is_closed = false;

    return 0;
}

bool contactor::get_armed()
{
    return is_armed;
}
bool contactor::get_status()
{
    return is_closed;
}
int contactor::close()
{
    __ASSERT(is_armed, "Attempted to close Contactor before Arming. Not allowed");
    if (is_faulted)
    {
        LOG_ERR("Contactor is Faulted, Cannot close Contactor after a fault");
        return -1;
    }
    gpio_ref.set(true);
    LOG_INF("Contactor closed");
    is_armed = false;
    is_closed = true;
    return 0;
}

int contactor::throw_fault()
{
    LOG_ERR("Contactor has been placed into fault state. Cannot be cleared without restarting vehicle");
    std::ignore = this->open();
    is_faulted = true;
    return 0;
}

#include "AIR.h"
#include "zephyr/drivers/gpio.h"

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
        // swap to enum type
        return -1;
    }
    is_armed = true;

    return 0;
}

int contactor::disarm()
{
    is_armed = false;
    return 0;
}
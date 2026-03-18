#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "threads/APPS.h"
#include "hardware.h"
#include "threads/Logger.h"
#include "threads/system.h"
#include "vehicle_state.h"

LOG_MODULE_REGISTER(main);

int main(void)
{
    LOG_INF("***VCU ENTERED MAIN***");

    static VehicleState vehicle;
    static Hardware hardware(&vehicle);
    static System system;

    LOG_INF("=== VCU Starting ===");

    if (system.init() != 0)
    {
        LOG_ERR("System init failed!");
        return -1;
    }

    // before initializing any threads we guarantee that hardware has initialized and APIs are available
    if (hardware.init() != 0)
    {
        LOG_ERR("Hardware init failed!");
        return -2;
    }

    // Start the APPS pedal processing task (100 ms period, priority 5).
    start_apps_task(&vehicle, &hardware);

    // Start the system diagnostics task (1000 ms period, priority 10).
    start_diagnostics_task(&system, &hardware, &vehicle);

    // starts logger task that broadcasts data from vehicle_state onto CANBUS2
    start_logger_task(&system, &hardware, &vehicle);

    LOG_INF("=== VCU Ready ===");

    while (1)
    {
        k_sleep(K_FOREVER);
    }
}

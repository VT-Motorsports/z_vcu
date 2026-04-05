#pragma once
#include "gpio.h"
//    gpioa_ = DEVICE_DT_GET(DT_NODELABEL(gpioa));

class contactor
{
  public:
    contactor();
    ~contactor() = default;

    [[nodiscard("Do not discard Contactor logic returns")]] int init();
    [[nodiscard("Do not discard Contactor logic returns")]] int arm();
    [[nodiscard("Do not discard Contactor logic returns")]] int disarm();
    [[nodiscard("Do not discard Contactor logic returns")]] int close();
    [[nodiscard("Do not discard Contactor logic returns")]] int open();
    [[nodiscard("Do not discard Contactor logic returns")]] bool get_armed();
    [[nodiscard("Do not discard Contactor logic returns")]] bool get_status();
    [[nodiscard("Do not discard Contactor logic returns")]] int throw_fault();

  private:
    bool is_armed = false;
    bool is_closed = false;
    bool is_faulted = false;
    const struct device *gpioa_ = nullptr;

    GpioPin gpio_ref;
};

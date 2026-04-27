#pragma once
#include "gpio.h"
#include "VSM_faults.h"

//    gpioa_ = DEVICE_DT_GET(DT_NODELABEL(gpioa));

class contactor {
  public:
    contactor();
    ~contactor() = default;

    [[nodiscard("Do not discard Contactor logic returns")]] int init();
    [[nodiscard("Do not discard Contactor logic returns")]] VSM_FAULTS arm();
    [[nodiscard("Do not discard Contactor logic returns")]] VSM_FAULTS disarm();
    [[nodiscard("Do not discard Contactor logic returns")]] VSM_FAULTS close();
    [[nodiscard("Do not discard Contactor logic returns")]] int open();
    [[nodiscard("Do not discard Contactor logic returns")]] bool get_armed() const;
    [[nodiscard("Do not discard Contactor logic returns")]] bool get_closed() const;
    [[nodiscard("Do not discard Contactor logic returns")]] VSM_FAULTS throw_fault();

  private:
    bool is_armed = false;
    bool is_closed = false;
    bool is_faulted = false;
    const struct device *gpioa_ = nullptr;

    GpioPin gpio_ref;
};

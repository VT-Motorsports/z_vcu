#include "gpio.h"
//    gpioa_ = DEVICE_DT_GET(DT_NODELABEL(gpioa));

class contactor
{
  public:
    contactor();
    ~contactor();

    int init();
    int arm();
    int disarm();

  private:
    bool is_armed = false;
    bool is_closed = false;
    bool is_faulted = false;
    const struct device *gpioa_ = nullptr;

    GpioPin gpio_ref;
};

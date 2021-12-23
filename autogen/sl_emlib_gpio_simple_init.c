#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_PB04_config.h"
#include "sl_emlib_gpio_init_blueLED_config.h"
#include "sl_emlib_gpio_init_gpio_config.h"
#include "sl_emlib_gpio_init_greenLED_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_PB04_PORT,
                  SL_EMLIB_GPIO_INIT_PB04_PIN,
                  SL_EMLIB_GPIO_INIT_PB04_MODE,
                  SL_EMLIB_GPIO_INIT_PB04_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BLUELED_PORT,
                  SL_EMLIB_GPIO_INIT_BLUELED_PIN,
                  SL_EMLIB_GPIO_INIT_BLUELED_MODE,
                  SL_EMLIB_GPIO_INIT_BLUELED_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_GPIO_PORT,
                  SL_EMLIB_GPIO_INIT_GPIO_PIN,
                  SL_EMLIB_GPIO_INIT_GPIO_MODE,
                  SL_EMLIB_GPIO_INIT_GPIO_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_GREENLED_PORT,
                  SL_EMLIB_GPIO_INIT_GREENLED_PIN,
                  SL_EMLIB_GPIO_INIT_GREENLED_MODE,
                  SL_EMLIB_GPIO_INIT_GREENLED_DOUT);
}

#include "app_main.h"
#include "ti_msp_dl_config.h"

int main(void)
{
  SYSCFG_DL_init();
  DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
  app_main();
  for (;;)
  {
  }
}

#include "app_main.h"
#include "ti_msp_dl_config.h"

static void MotorOutputsSafe(void)
{
  const uint32_t period = DL_Timer_getLoadValue(MOTOR_PWM_INST);

  DL_Timer_stopCounter(MOTOR_PWM_INST);
  DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST, period, DL_TIMER_CC_2_INDEX);
  DL_TimerA_setCaptureCompareValue(MOTOR_PWM_INST, period, DL_TIMER_CC_3_INDEX);
  DL_TimerA_setCCPOutputDisabledAdv(
      MOTOR_PWM_INST,
      DL_TIMERA_CCP2_DIS_OUT_ADV_FORCE_LOW | DL_TIMERA_CCP3_DIS_OUT_ADV_FORCE_LOW);

  DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_AIN1_PIN);
  DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_AIN2_PIN);
  DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_BIN1_PIN);
  DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_BIN2_PIN);
}

int main(void)
{
  SYSCFG_DL_init();
  MotorOutputsSafe();
  DL_SYSTICK_config(CPUCLK_FREQ / 1000U);
  app_main();
  for (;;)
  {
  }
}

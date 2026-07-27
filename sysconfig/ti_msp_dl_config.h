/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the LP_MSPM0G3507
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_LP_MSPM0G3507
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for MOTOR_PWM */
#define MOTOR_PWM_INST                                                     TIMA0
#define MOTOR_PWM_INST_IRQHandler                               TIMA0_IRQHandler
#define MOTOR_PWM_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define MOTOR_PWM_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 2 */
#define GPIO_MOTOR_PWM_C2_PORT                                             GPIOB
#define GPIO_MOTOR_PWM_C2_PIN                                     DL_GPIO_PIN_20
#define GPIO_MOTOR_PWM_C2_IOMUX                                  (IOMUX_PINCM48)
#define GPIO_MOTOR_PWM_C2_IOMUX_FUNC                 IOMUX_PINCM48_PF_TIMA0_CCP2
#define GPIO_MOTOR_PWM_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_MOTOR_PWM_C3_PORT                                             GPIOA
#define GPIO_MOTOR_PWM_C3_PIN                                     DL_GPIO_PIN_28
#define GPIO_MOTOR_PWM_C3_IOMUX                                   (IOMUX_PINCM3)
#define GPIO_MOTOR_PWM_C3_IOMUX_FUNC                  IOMUX_PINCM3_PF_TIMA0_CCP3
#define GPIO_MOTOR_PWM_C3_IDX                                DL_TIMER_CC_3_INDEX




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL

/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C1
#define I2C_1_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C1_INT_IRQn
#define I2C_1_BUS_SPEED_HZ                                                400000
#define GPIO_I2C_1_SDA_PORT                                                GPIOB
#define GPIO_I2C_1_SDA_PIN                                         DL_GPIO_PIN_3
#define GPIO_I2C_1_IOMUX_SDA                                     (IOMUX_PINCM16)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                      IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOB
#define GPIO_I2C_1_SCL_PIN                                         DL_GPIO_PIN_2
#define GPIO_I2C_1_IOMUX_SCL                                     (IOMUX_PINCM15)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                      IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                               (2000000)
#define UART_0_IBRD_32_MHZ_2000000_BAUD                                      (1)
#define UART_0_FBRD_32_MHZ_2000000_BAUD                                      (0)
/* Defines for DMA_CH_TX */
#define DMA_CH_TX_CHAN_ID                                                    (0)
#define DMA_CH_TX_TRIGGER_SEL_SW                             (DMA_SOFTWARE_TRIG)
/* Defines for DMA_CH_RX */
#define DMA_CH_RX_CHAN_ID                                                    (1)
#define DMA_CH_RX_TRIGGER_SEL_SW                             (DMA_SOFTWARE_TRIG)


/* Port definition for Pin Group GPIO_GRP_0 */
#define GPIO_GRP_0_PORT                                                  (GPIOB)

/* Defines for PIN_0: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_GRP_0_PIN_0_PIN                                    (DL_GPIO_PIN_22)
#define GPIO_GRP_0_PIN_0_IOMUX                                   (IOMUX_PINCM50)
/* Port definition for Pin Group MOTOR_AIN1 */
#define MOTOR_AIN1_PORT                                                  (GPIOA)

/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define MOTOR_AIN1_AIN1_PIN                                     (DL_GPIO_PIN_13)
#define MOTOR_AIN1_AIN1_IOMUX                                    (IOMUX_PINCM35)
/* Port definition for Pin Group MOTOR_AIN2 */
#define MOTOR_AIN2_PORT                                                  (GPIOB)

/* Defines for AIN2: GPIOB.26 with pinCMx 57 on package pin 28 */
#define MOTOR_AIN2_AIN2_PIN                                     (DL_GPIO_PIN_26)
#define MOTOR_AIN2_AIN2_IOMUX                                    (IOMUX_PINCM57)
/* Port definition for Pin Group MOTOR_BIN1 */
#define MOTOR_BIN1_PORT                                                  (GPIOB)

/* Defines for BIN1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define MOTOR_BIN1_BIN1_PIN                                      (DL_GPIO_PIN_9)
#define MOTOR_BIN1_BIN1_IOMUX                                    (IOMUX_PINCM26)
/* Port definition for Pin Group MOTOR_BIN2 */
#define MOTOR_BIN2_PORT                                                  (GPIOB)

/* Defines for BIN2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define MOTOR_BIN2_BIN2_PIN                                      (DL_GPIO_PIN_7)
#define MOTOR_BIN2_BIN2_IOMUX                                    (IOMUX_PINCM24)
/* Port definition for Pin Group ICM42688_INT */
#define ICM42688_INT_PORT                                                (GPIOB)

/* Defines for INT1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define ICM42688_INT_INT1_PIN                                    (DL_GPIO_PIN_6)
#define ICM42688_INT_INT1_IOMUX                                  (IOMUX_PINCM23)
/* Port definition for Pin Group ENCODERS */
#define ENCODERS_PORT                                                    (GPIOB)

/* Defines for E1A: GPIOB.23 with pinCMx 51 on package pin 22 */
#define ENCODERS_E1A_PIN                                        (DL_GPIO_PIN_23)
#define ENCODERS_E1A_IOMUX                                       (IOMUX_PINCM51)
/* Defines for E1B: GPIOB.12 with pinCMx 29 on package pin 64 */
#define ENCODERS_E1B_PIN                                        (DL_GPIO_PIN_12)
#define ENCODERS_E1B_IOMUX                                       (IOMUX_PINCM29)
/* Defines for E2A: GPIOB.4 with pinCMx 17 on package pin 52 */
#define ENCODERS_E2A_PIN                                         (DL_GPIO_PIN_4)
#define ENCODERS_E2A_IOMUX                                       (IOMUX_PINCM17)
/* Defines for E2B: GPIOB.5 with pinCMx 18 on package pin 53 */
#define ENCODERS_E2B_PIN                                         (DL_GPIO_PIN_5)
#define ENCODERS_E2B_IOMUX                                       (IOMUX_PINCM18)
/* Port definition for Pin Group LINE_A */
#define LINE_A_PORT                                                      (GPIOA)

/* Defines for AD1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define LINE_A_AD1_PIN                                          (DL_GPIO_PIN_12)
#define LINE_A_AD1_IOMUX                                         (IOMUX_PINCM34)
/* Defines for AD6: GPIOA.16 with pinCMx 38 on package pin 9 */
#define LINE_A_AD6_PIN                                          (DL_GPIO_PIN_16)
#define LINE_A_AD6_IOMUX                                         (IOMUX_PINCM38)
/* Defines for AD7: GPIOA.24 with pinCMx 54 on package pin 25 */
#define LINE_A_AD7_PIN                                          (DL_GPIO_PIN_24)
#define LINE_A_AD7_IOMUX                                         (IOMUX_PINCM54)
/* Defines for AD8: GPIOA.25 with pinCMx 55 on package pin 26 */
#define LINE_A_AD8_PIN                                          (DL_GPIO_PIN_25)
#define LINE_A_AD8_IOMUX                                         (IOMUX_PINCM55)
/* Port definition for Pin Group LINE_B */
#define LINE_B_PORT                                                      (GPIOB)

/* Defines for AD2: GPIOB.8 with pinCMx 25 on package pin 60 */
#define LINE_B_AD2_PIN                                           (DL_GPIO_PIN_8)
#define LINE_B_AD2_IOMUX                                         (IOMUX_PINCM25)
/* Defines for AD3: GPIOB.21 with pinCMx 49 on package pin 20 */
#define LINE_B_AD3_PIN                                          (DL_GPIO_PIN_21)
#define LINE_B_AD3_IOMUX                                         (IOMUX_PINCM49)
/* Defines for AD4: GPIOB.19 with pinCMx 45 on package pin 16 */
#define LINE_B_AD4_PIN                                          (DL_GPIO_PIN_19)
#define LINE_B_AD4_IOMUX                                         (IOMUX_PINCM45)
/* Defines for AD5: GPIOB.17 with pinCMx 43 on package pin 14 */
#define LINE_B_AD5_PIN                                          (DL_GPIO_PIN_17)
#define LINE_B_AD5_IOMUX                                         (IOMUX_PINCM43)
/* Defines for KEY1: GPIOB.25 with pinCMx 56 on package pin 27 */
#define KEYS_KEY1_PORT                                                   (GPIOB)
#define KEYS_KEY1_PIN                                           (DL_GPIO_PIN_25)
#define KEYS_KEY1_IOMUX                                          (IOMUX_PINCM56)
/* Defines for KEY2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define KEYS_KEY2_PORT                                                   (GPIOA)
#define KEYS_KEY2_PIN                                           (DL_GPIO_PIN_14)
#define KEYS_KEY2_IOMUX                                          (IOMUX_PINCM36)
/* Defines for KEY3: GPIOB.24 with pinCMx 52 on package pin 23 */
#define KEYS_KEY3_PORT                                                   (GPIOB)
#define KEYS_KEY3_PIN                                           (DL_GPIO_PIN_24)
#define KEYS_KEY3_IOMUX                                          (IOMUX_PINCM52)
/* Defines for KEY4: GPIOA.15 with pinCMx 37 on package pin 8 */
#define KEYS_KEY4_PORT                                                   (GPIOA)
#define KEYS_KEY4_PIN                                           (DL_GPIO_PIN_15)
#define KEYS_KEY4_IOMUX                                          (IOMUX_PINCM37)
/* Defines for LED1: GPIOA.21 with pinCMx 46 on package pin 17 */
#define LEDS_LED1_PORT                                                   (GPIOA)
#define LEDS_LED1_PIN                                           (DL_GPIO_PIN_21)
#define LEDS_LED1_IOMUX                                          (IOMUX_PINCM46)
/* Defines for LED2: GPIOB.18 with pinCMx 44 on package pin 15 */
#define LEDS_LED2_PORT                                                   (GPIOB)
#define LEDS_LED2_PIN                                           (DL_GPIO_PIN_18)
#define LEDS_LED2_IOMUX                                          (IOMUX_PINCM44)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_MOTOR_PWM_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */

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
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
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


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for motor_PWM */
#define motor_PWM_INST                                                     TIMA0
#define motor_PWM_INST_IRQHandler                               TIMA0_IRQHandler
#define motor_PWM_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define motor_PWM_INST_CLK_FREQ                                         40000000
/* GPIO defines for channel 0 */
#define GPIO_motor_PWM_C0_PORT                                             GPIOB
#define GPIO_motor_PWM_C0_PIN                                      DL_GPIO_PIN_8
#define GPIO_motor_PWM_C0_IOMUX                                  (IOMUX_PINCM25)
#define GPIO_motor_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM25_PF_TIMA0_CCP0
#define GPIO_motor_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_motor_PWM_C1_PORT                                             GPIOB
#define GPIO_motor_PWM_C1_PIN                                     DL_GPIO_PIN_12
#define GPIO_motor_PWM_C1_IOMUX                                  (IOMUX_PINCM29)
#define GPIO_motor_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM29_PF_TIMA0_CCP1
#define GPIO_motor_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_motor_PWM_C2_PORT                                             GPIOA
#define GPIO_motor_PWM_C2_PIN                                      DL_GPIO_PIN_3
#define GPIO_motor_PWM_C2_IOMUX                                   (IOMUX_PINCM8)
#define GPIO_motor_PWM_C2_IOMUX_FUNC                  IOMUX_PINCM8_PF_TIMA0_CCP2
#define GPIO_motor_PWM_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_motor_PWM_C3_PORT                                             GPIOB
#define GPIO_motor_PWM_C3_PIN                                      DL_GPIO_PIN_2
#define GPIO_motor_PWM_C3_IOMUX                                  (IOMUX_PINCM15)
#define GPIO_motor_PWM_C3_IOMUX_FUNC                 IOMUX_PINCM15_PF_TIMA0_CCP3
#define GPIO_motor_PWM_C3_IDX                                DL_TIMER_CC_3_INDEX



/* Defines for TIMER_20ms */
#define TIMER_20ms_INST                                                  (TIMG0)
#define TIMER_20ms_INST_IRQHandler                              TIMG0_IRQHandler
#define TIMER_20ms_INST_INT_IRQN                                (TIMG0_INT_IRQn)
#define TIMER_20ms_INST_LOAD_VALUE                                       (9999U)
/* Defines for TIMER_1ms */
#define TIMER_1ms_INST                                                   (TIMA1)
#define TIMER_1ms_INST_IRQHandler                               TIMA1_IRQHandler
#define TIMER_1ms_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define TIMER_1ms_INST_LOAD_VALUE                                         (799U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           10000000
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
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_10_MHZ_115200_BAUD                                       (5)
#define UART_0_FBRD_10_MHZ_115200_BAUD                                      (27)




/* Defines for SPI_W25Q64 */
#define SPI_W25Q64_INST                                                    SPI1
#define SPI_W25Q64_INST_IRQHandler                              SPI1_IRQHandler
#define SPI_W25Q64_INST_INT_IRQN                                  SPI1_INT_IRQn
#define GPIO_SPI_W25Q64_PICO_PORT                                         GPIOB
#define GPIO_SPI_W25Q64_PICO_PIN                                 DL_GPIO_PIN_15
#define GPIO_SPI_W25Q64_IOMUX_PICO                              (IOMUX_PINCM32)
#define GPIO_SPI_W25Q64_IOMUX_PICO_FUNC              IOMUX_PINCM32_PF_SPI1_PICO
#define GPIO_SPI_W25Q64_POCI_PORT                                         GPIOB
#define GPIO_SPI_W25Q64_POCI_PIN                                 DL_GPIO_PIN_14
#define GPIO_SPI_W25Q64_IOMUX_POCI                              (IOMUX_PINCM31)
#define GPIO_SPI_W25Q64_IOMUX_POCI_FUNC              IOMUX_PINCM31_PF_SPI1_POCI
/* GPIO configuration for SPI_W25Q64 */
#define GPIO_SPI_W25Q64_SCLK_PORT                                         GPIOB
#define GPIO_SPI_W25Q64_SCLK_PIN                                 DL_GPIO_PIN_16
#define GPIO_SPI_W25Q64_IOMUX_SCLK                              (IOMUX_PINCM33)
#define GPIO_SPI_W25Q64_IOMUX_SCLK_FUNC              IOMUX_PINCM33_PF_SPI1_SCLK



/* Defines for ADC_Senor */
#define ADC_Senor_INST                                                      ADC0
#define ADC_Senor_INST_IRQHandler                                ADC0_IRQHandler
#define ADC_Senor_INST_INT_IRQN                                  (ADC0_INT_IRQn)
#define ADC_Senor_ADCMEM_IR_left                              DL_ADC12_MEM_IDX_0
#define ADC_Senor_ADCMEM_IR_left_REF             DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_Senor_ADCMEM_IR_left_REF_VOLTAGE_V                                     3.3
#define ADC_Senor_ADCMEM_IR_right                             DL_ADC12_MEM_IDX_1
#define ADC_Senor_ADCMEM_IR_right_REF            DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_Senor_ADCMEM_IR_right_REF_VOLTAGE_V                                     3.3
#define ADC_Senor_ADCMEM_light_left                           DL_ADC12_MEM_IDX_2
#define ADC_Senor_ADCMEM_light_left_REF          DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_Senor_ADCMEM_light_left_REF_VOLTAGE_V                                     3.3
#define ADC_Senor_ADCMEM_light_right                          DL_ADC12_MEM_IDX_3
#define ADC_Senor_ADCMEM_light_right_REF         DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_Senor_ADCMEM_light_right_REF_VOLTAGE_V                                     3.3
#define ADC_Senor_ADCMEM_4                                    DL_ADC12_MEM_IDX_4
#define ADC_Senor_ADCMEM_4_REF                   DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_Senor_ADCMEM_4_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC_Senor_C2_PORT                                             GPIOA
#define GPIO_ADC_Senor_C2_PIN                                     DL_GPIO_PIN_25
#define GPIO_ADC_Senor_IOMUX_C2                                  (IOMUX_PINCM55)
#define GPIO_ADC_Senor_IOMUX_C2_FUNC              (IOMUX_PINCM55_PF_UNCONNECTED)
#define GPIO_ADC_Senor_C4_PORT                                             GPIOB
#define GPIO_ADC_Senor_C4_PIN                                     DL_GPIO_PIN_25
#define GPIO_ADC_Senor_IOMUX_C4                                  (IOMUX_PINCM56)
#define GPIO_ADC_Senor_IOMUX_C4_FUNC              (IOMUX_PINCM56_PF_UNCONNECTED)
#define GPIO_ADC_Senor_C5_PORT                                             GPIOB
#define GPIO_ADC_Senor_C5_PIN                                     DL_GPIO_PIN_24
#define GPIO_ADC_Senor_IOMUX_C5                                  (IOMUX_PINCM52)
#define GPIO_ADC_Senor_IOMUX_C5_FUNC              (IOMUX_PINCM52_PF_UNCONNECTED)
#define GPIO_ADC_Senor_C3_PORT                                             GPIOA
#define GPIO_ADC_Senor_C3_PIN                                     DL_GPIO_PIN_24
#define GPIO_ADC_Senor_IOMUX_C3                                  (IOMUX_PINCM54)
#define GPIO_ADC_Senor_IOMUX_C3_FUNC              (IOMUX_PINCM54_PF_UNCONNECTED)
#define GPIO_ADC_Senor_C0_PORT                                             GPIOA
#define GPIO_ADC_Senor_C0_PIN                                     DL_GPIO_PIN_27
#define GPIO_ADC_Senor_IOMUX_C0                                  (IOMUX_PINCM60)
#define GPIO_ADC_Senor_IOMUX_C0_FUNC              (IOMUX_PINCM60_PF_UNCONNECTED)



/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for MCU: GPIOA.17 with pinCMx 39 on package pin 10 */
#define LED_MCU_PIN                                             (DL_GPIO_PIN_17)
#define LED_MCU_IOMUX                                            (IOMUX_PINCM39)
/* Port definition for Pin Group BEEP */
#define BEEP_PORT                                                        (GPIOB)

/* Defines for Buzzer: GPIOB.5 with pinCMx 18 on package pin 53 */
#define BEEP_Buzzer_PIN                                          (DL_GPIO_PIN_5)
#define BEEP_Buzzer_IOMUX                                        (IOMUX_PINCM18)
/* Port definition for Pin Group RGB */
#define RGB_PORT                                                         (GPIOB)

/* Defines for WQ2812: GPIOB.10 with pinCMx 27 on package pin 62 */
#define RGB_WQ2812_PIN                                          (DL_GPIO_PIN_10)
#define RGB_WQ2812_IOMUX                                         (IOMUX_PINCM27)
/* Port definition for Pin Group Infrared_borad */
#define Infrared_borad_PORT                                              (GPIOA)

/* Defines for IR_switch: GPIOA.9 with pinCMx 20 on package pin 55 */
#define Infrared_borad_IR_switch_PIN                             (DL_GPIO_PIN_9)
#define Infrared_borad_IR_switch_IOMUX                           (IOMUX_PINCM20)
/* Port definition for Pin Group IRContorl */
#define IRContorl_PORT                                                   (GPIOB)

/* Defines for GET_OUT: GPIOB.3 with pinCMx 16 on package pin 51 */
// groups represented: ["GPIO_ENCODER_L","GPIO_ENCODER_R","IRContorl"]
// pins affected: ["H1A","H1B","H2A","GET_OUT"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define IRContorl_GET_OUT_IIDX                               (DL_GPIO_IIDX_DIO3)
#define IRContorl_GET_OUT_PIN                                    (DL_GPIO_PIN_3)
#define IRContorl_GET_OUT_IOMUX                                  (IOMUX_PINCM16)
/* Port definition for Pin Group SPI */
#define SPI_PORT                                                         (GPIOB)

/* Defines for CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define SPI_CS_PIN                                               (DL_GPIO_PIN_6)
#define SPI_CS_IOMUX                                             (IOMUX_PINCM23)
/* Defines for button1: GPIOA.18 with pinCMx 40 on package pin 11 */
#define KEY_button1_PORT                                                 (GPIOA)
#define KEY_button1_PIN                                         (DL_GPIO_PIN_18)
#define KEY_button1_IOMUX                                        (IOMUX_PINCM40)
/* Defines for button2: GPIOB.23 with pinCMx 51 on package pin 22 */
#define KEY_button2_PORT                                                 (GPIOB)
#define KEY_button2_PIN                                         (DL_GPIO_PIN_23)
#define KEY_button2_IOMUX                                        (IOMUX_PINCM51)
/* Defines for button3: GPIOB.20 with pinCMx 48 on package pin 19 */
#define KEY_button3_PORT                                                 (GPIOB)
#define KEY_button3_PIN                                         (DL_GPIO_PIN_20)
#define KEY_button3_IOMUX                                        (IOMUX_PINCM48)
/* Port definition for Pin Group GPIO_ENCODER_L */
#define GPIO_ENCODER_L_PORT                                              (GPIOB)

/* Defines for H1A: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_ENCODER_L_H1A_IIDX                             (DL_GPIO_IIDX_DIO21)
#define GPIO_ENCODER_L_H1A_PIN                                  (DL_GPIO_PIN_21)
#define GPIO_ENCODER_L_H1A_IOMUX                                 (IOMUX_PINCM49)
/* Defines for H1B: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_ENCODER_L_H1B_IIDX                             (DL_GPIO_IIDX_DIO22)
#define GPIO_ENCODER_L_H1B_PIN                                  (DL_GPIO_PIN_22)
#define GPIO_ENCODER_L_H1B_IOMUX                                 (IOMUX_PINCM50)
/* Defines for H2A: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_ENCODER_R_H2A_PORT                                          (GPIOB)
#define GPIO_ENCODER_R_H2A_IIDX                             (DL_GPIO_IIDX_DIO13)
#define GPIO_ENCODER_R_H2A_PIN                                  (DL_GPIO_PIN_13)
#define GPIO_ENCODER_R_H2A_IOMUX                                 (IOMUX_PINCM30)
/* Defines for H2B: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_ENCODER_R_H2B_PORT                                          (GPIOA)
// pins affected by this interrupt request:["H2B"]
#define GPIO_ENCODER_R_GPIOA_INT_IRQN                           (GPIOA_INT_IRQn)
#define GPIO_ENCODER_R_GPIOA_INT_IIDX           (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODER_R_H2B_IIDX                             (DL_GPIO_IIDX_DIO31)
#define GPIO_ENCODER_R_H2B_PIN                                  (DL_GPIO_PIN_31)
#define GPIO_ENCODER_R_H2B_IOMUX                                  (IOMUX_PINCM6)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOA)

/* Defines for SCL1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define OLED_SCL1_PIN                                           (DL_GPIO_PIN_15)
#define OLED_SCL1_IOMUX                                          (IOMUX_PINCM37)
/* Defines for SDA1: GPIOA.30 with pinCMx 5 on package pin 37 */
#define OLED_SDA1_PIN                                           (DL_GPIO_PIN_30)
#define OLED_SDA1_IOMUX                                           (IOMUX_PINCM5)
/* Port definition for Pin Group MPU6050 */
#define MPU6050_PORT                                                     (GPIOA)

/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define MPU6050_SDA_PIN                                          (DL_GPIO_PIN_0)
#define MPU6050_SDA_IOMUX                                         (IOMUX_PINCM1)
/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define MPU6050_SCL_PIN                                          (DL_GPIO_PIN_1)
#define MPU6050_SCL_IOMUX                                         (IOMUX_PINCM2)




/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_motor_PWM_init(void);
void SYSCFG_DL_TIMER_20ms_init(void);
void SYSCFG_DL_TIMER_1ms_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_W25Q64_init(void);
void SYSCFG_DL_ADC_Senor_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */

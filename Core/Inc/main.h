/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC_IN_NOT_IN_USE_Pin GPIO_PIN_0
#define OSC_IN_NOT_IN_USE_GPIO_Port GPIOF
#define OSC_OUT_NOT_IN_USE_Pin GPIO_PIN_1
#define OSC_OUT_NOT_IN_USE_GPIO_Port GPIOF
#define OUTPUT_CURRENT_Pin GPIO_PIN_0
#define OUTPUT_CURRENT_GPIO_Port GPIOA
#define LO_POW_SUPPLY_Pin GPIO_PIN_1
#define LO_POW_SUPPLY_GPIO_Port GPIOA
#define HI_POW_SUPPLY_Pin GPIO_PIN_2
#define HI_POW_SUPPLY_GPIO_Port GPIOA
#define OUTPUT_VOLTAGE_Pin GPIO_PIN_3
#define OUTPUT_VOLTAGE_GPIO_Port GPIOA
#define OUTPUT_OVERLOAD_Pin GPIO_PIN_4
#define OUTPUT_OVERLOAD_GPIO_Port GPIOA
#define PA5_NOT_IN_USE_Pin GPIO_PIN_5
#define PA5_NOT_IN_USE_GPIO_Port GPIOA
#define PWM_LO_POW_UPPER_GATE_Pin GPIO_PIN_6
#define PWM_LO_POW_UPPER_GATE_GPIO_Port GPIOA
#define PWM_LOWER_GATES_Pin GPIO_PIN_7
#define PWM_LOWER_GATES_GPIO_Port GPIOA
#define PWM_HI_POW_UPPER_GATE_Pin GPIO_PIN_1
#define PWM_HI_POW_UPPER_GATE_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_9
#define STATUS_LED_GPIO_Port GPIOA
#define PIN10_NOT_IN_USE_Pin GPIO_PIN_10
#define PIN10_NOT_IN_USE_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/*** GENERAL SETTINGS BEGIN ***/

// Disables the output by forcing PWM pulse width to 0
//#define DISABLE_OUT
// 4 channels, no averaging
#define ADC_DMA_BUFFER_SIZE 4
// Power supply nominal voltage, V
#define POW_SUP_NOM         24
// Power supply maximum voltage, V
#define POW_SUP_MAX         32
// Power supply minimum voltage, V
#define POW_SUP_MIN         19

// Lower threshold to recognise the monitored supply channel as dead, V
#define POW_HYST_LTHLD      19
// Lower threshold to recognise the monitored supply channel as live, V
#define POW_HYST_UTHLD      21
// Output current threshold to switch to current limiter, A
#define LOAD_CUR_UTHLD      17
// Minimum load voltage below which a short circuit event is initiated
// This setting acts in conjunction with LOAD_SC_CUR
#define LOAD_SC_VOLT        5
// Maximum load current abive which a short circuit event is initiated
// This setting acts in conjunction with LOAD_SC_VOLT
#define LOAD_SC_CUR         10

// PWM period, Hz
#define PWM_CLK_FREQ        48000
// TIM clock, Hz
#define TIM_CLK_FREQ        48000000
// Moving average window
#define MOV_AVG_WINDOW      20

// PID sampling period expressed in the number of TIM3 periods
#define PID_PERIOD_CODE		10
// Voltage loop set point in DIMMED mode, V
#define PID_VLP_DIM_SP      12.25
// Voltage loop set point in BRIGHT mode, V
#define PID_VLP_BRT_SP      24.75
// Current loop set point in DIMMED mode, V
#define PID_CLP_DIM_SP      9
// Current loop set point in BRIGHT mode, V
#define PID_CLP_BRT_SP      13
// Voltage loop PID proportional coefficient
#define PID_VLP_KP          10
// Voltage loop PID integral coefficient
#define PID_VLP_KI          20
// Current loop PID proportional coefficient
#define PID_CLP_KP          50
// Current loop PID integral coefficient
#define PID_CLP_KI          50
// Current loop PID deifferential coefficient
#define PID_CLP_KD          75
// PID pulse width minimum code
#define PID_MIN             0
// PID pulse width maximum code
#define PID_MAX             4095
// PID-to-PWM pulse width transfer coefficient (PWM = PID / PID_TO_PWM)
#define PID_TO_PWM          8

// PWM dead time
#define PWM_DEAD_TIME       25

// Overload delay
#define OVERLOAD_DLY        240000
// Overload timeout before testing the load
#define OVLD_TIMEOUT        32000
//
#define START_UP_DLY        32000
//
#define LOAD_TEST_DLY       100
//
#define NUM_OF_ATTEMPTS_TO_START 5

/*** GENERAL SETTINGS END ***/



/*** MACROS BEGIN ***/
// Coefficient to transfer from real values expressed in Volts to ADC codes
#define ADC_SUP_VOLT_TO_CODE 117
// Coefficient to transfer from real values expressed in Amperes to ADC codes
#define ADC_LOAD_CUR_TO_CODE 117

// The following macros express device settings of supply voltage and load
// current thresholds in ADC codes
// Minimum, maximum and nominal supply voltages
#define POW_SUP_MIN_CODE    ((uint16_t)(POW_SUP_MIN * ADC_SUP_VOLT_TO_CODE))
#define POW_SUP_MAX_CODE    ((uint16_t)(POW_SUP_MAX * ADC_SUP_VOLT_TO_CODE))
#define POW_SUP_NOM_CODE    ((uint16_t)(POW_SUP_NOM * ADC_SUP_VOLT_TO_CODE))
// Suppply voltage thresholds used in selection of active supply channel
#define POW_HYST_LTHLD_CODE ((uint16_t)(POW_HYST_LTHLD * ADC_SUP_VOLT_TO_CODE))
#define POW_HYST_UTHLD_CODE ((uint16_t)(POW_HYST_UTHLD * ADC_SUP_VOLT_TO_CODE))

// Upper load current threshold to detect overload
#define LOAD_CUR_UTHLD_CODE ((uint16_t)(LOAD_CUR_UTHLD * ADC_LOAD_CUR_TO_CODE))
// Voltage loop PID set points for DIMMED and BRIGTH mode
#define PID_VLP_DIM_SP_CODE ((int16_t)(PID_VLP_DIM_SP * ADC_SUP_VOLT_TO_CODE))
#define PID_VLP_BRT_SP_CODE ((int16_t)(PID_VLP_BRT_SP * ADC_SUP_VOLT_TO_CODE))
// Current loop PID set points for DIMMED and BRIGTH mode
#define PID_CLP_DIM_SP_CODE ((int16_t)(PID_CLP_DIM_SP * ADC_LOAD_CUR_TO_CODE))
#define PID_CLP_BRT_SP_CODE ((int16_t)(PID_CLP_BRT_SP * ADC_LOAD_CUR_TO_CODE))
// Short circuit critical values of load cuurent and voltage
#define LOAD_SC_VOLT_CODE   ((uint16_t)(LOAD_SC_VOLT * ADC_SUP_VOLT_TO_CODE))
#define LOAD_SC_CUR_CODE    ((uint16_t)(LOAD_SC_CUR * ADC_LOAD_CUR_TO_CODE))

// Service macros
// TIM3 is in center-aligned mode, so the register value has to be halved.
// Given zero presacler, the period is as follows:
#define PWM_PERIOD_CODE		((uint16_t)(0.5f * TIM_CLK_FREQ / PWM_CLK_FREQ))
// Returns absolute value for a given operand
#define ABS(X)              ((X) > 0 ? (X) : -(X))
// Convert from pid pulse width to pwm pulse width
#define GET_PWM_PULSE(X)    ((uint16_t)((X) / PID_TO_PWM))
// Verify pid pulse width against maximum allowed value
#define PID_CHECK_UTHLD(X)  ((uint16_t)((X) > PID_MAX ? PID_MAX : (X)))
// Verify pid pulse width against minimum allowed value
#define PID_CHECK_LTHLD(X)  ((uint16_t)((X) < PID_MIN ? PID_MIN : (X)))
// Verify pwm pulse width against maximum allowed value
#define PWM_CHECK_UTHLD(X)  ((uint16_t)((X) > (PWM_PERIOD_CODE - PWM_DEAD_TIME) ? \
                            (PWM_PERIOD_CODE - PWM_DEAD_TIME) : (X)))
// Verify pwm pulse width against minimum allowed value
#define PWM_CHECK_LTHLD(X)  ((uint16_t)((X) < PWM_DEAD_TIME ? 0 : (X)))

/*** MACROS END ***/

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

	/*
	 * 	USB config files:
	 * 		usbd_customhid.h
	 * 		usbd_custom_hid.c	- This is where the USB descriptor is
	 * 		usbd_conf.h			- This is where descriptor size is
	 * 		usbd_custom_hid_if.c- This is where the USB report is
	 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
	uint8_t buttons;
	int16_t roll;
	int16_t pitch;
	uint8_t size;
}cyclic_report_t;

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
#define SW_OK_Pin GPIO_PIN_2
#define SW_OK_GPIO_Port GPIOE
#define SW_CAL_Pin GPIO_PIN_3
#define SW_CAL_GPIO_Port GPIOE
#define SW_CAL_EXTI_IRQn EXTI3_IRQn
#define LED_1_Pin GPIO_PIN_4
#define LED_1_GPIO_Port GPIOE
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOE
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOE
#define LED_4_Pin GPIO_PIN_13
#define LED_4_GPIO_Port GPIOC
#define UART4_EN_Pin GPIO_PIN_3
#define UART4_EN_GPIO_Port GPIOC
#define UART2_EN_Pin GPIO_PIN_4
#define UART2_EN_GPIO_Port GPIOA
#define CYCLIC_PITCH_Pin GPIO_PIN_4
#define CYCLIC_PITCH_GPIO_Port GPIOC
#define CYCLIC_ROLL_Pin GPIO_PIN_5
#define CYCLIC_ROLL_GPIO_Port GPIOC
#define CYC_ICS_SW_Pin GPIO_PIN_9
#define CYC_ICS_SW_GPIO_Port GPIOD
#define CYC_CARGO_REL_Pin GPIO_PIN_10
#define CYC_CARGO_REL_GPIO_Port GPIOD
#define CYC_CDR_REL_Pin GPIO_PIN_11
#define CYC_CDR_REL_GPIO_Port GPIOD
#define CYC_AP_REL_Pin GPIO_PIN_12
#define CYC_AP_REL_GPIO_Port GPIOD
#define CYC_HAT_LEFT_Pin GPIO_PIN_13
#define CYC_HAT_LEFT_GPIO_Port GPIOD
#define CYC_HAT_DOWN_Pin GPIO_PIN_14
#define CYC_HAT_DOWN_GPIO_Port GPIOD
#define CYC_HAT_UP_Pin GPIO_PIN_15
#define CYC_HAT_UP_GPIO_Port GPIOD
#define CYC_HAT_RIGHT_Pin GPIO_PIN_6
#define CYC_HAT_RIGHT_GPIO_Port GPIOC
#define FLASH_CS_Pin GPIO_PIN_7
#define FLASH_CS_GPIO_Port GPIOD
#define PITCH_STEP_PUL_Pin GPIO_PIN_6
#define PITCH_STEP_PUL_GPIO_Port GPIOB
#define PITCH_STEP_DIR_Pin GPIO_PIN_7
#define PITCH_STEP_DIR_GPIO_Port GPIOB
#define PITCH_STEP_EN_Pin GPIO_PIN_8
#define PITCH_STEP_EN_GPIO_Port GPIOB
#define ROLL_STEP_PUL_Pin GPIO_PIN_9
#define ROLL_STEP_PUL_GPIO_Port GPIOB
#define ROLL_STEP_DIR_Pin GPIO_PIN_0
#define ROLL_STEP_DIR_GPIO_Port GPIOE
#define ROLL_STEP_EN_Pin GPIO_PIN_1
#define ROLL_STEP_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define UINT16_T_MAX				(65536)
#define NUM_ADC_CHANNEL 			(2)
#define ROLL_ALPHA 					(0.3f)
#define PITCH_ALPHA					(0.3f)
#define CYCLIC_BUTTON_MASK			(0x7F)
#define AXIS_RANGE					(1023)
#define FLASH_CALIBRATION_ADDRESS	((uint32_t)0x020000)
#define CDR_BIT(x)					((x&(1<<7)) >> 7)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

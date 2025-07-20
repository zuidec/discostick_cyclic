/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_hid_custom2_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t buffer2[0x40];
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID2_ReportDesc[USBD_CUSTOM_HID2_REPORT_DESC_SIZE] __ALIGN_END =
    {
        /* USER CODE BEGIN 0 */
	0x05, 0x01,        /* Usage Page (Generic Desktop Ctrls)     */
	0x09, 0x04,        /* Usage (Joystick)                       */
	0xA1, 0x01,        /* Collection (Application)               */
		  0xA1, 0x00,        /*   Collection (Physical)              */
		  	  0x85, 0x02,		 	/*   ReportID (2)						 */
			  0x05, 0x01,        /*     Usage Page (Generic Desktop Ctrls) */
			  0x09, 0x32,        /*     Usage (Z)                          */
			  0x16, 0x01, 0xFC,  /*     Logical Minimum (-1023)            */
			  0x26, 0xFF, 0x03,  /*     Logical Maximum (1023)             */
			  0x75, 0x10,        /*     Report Size (16)                   */
			  0x95, 0x01,        /*     Report Count (1)                   */
			  0x81, 0x02,        /*     Input (Data,Var, Abs)               */

              0x09, 0x33,        /*     Usage (Rx)                          */
              0x09, 0x34,        /*     Usage (Ry)                          */
			  0x16, 0x00, 0x00,  /*     Logical Minimum (0)                 */
			  0x26, 0xFF, 0x07,  /*     Logical Maximum (2047)             */
			  0x75, 0x10,        /*     Report Size (16)                   */
			  0x95, 0x02,        /*     Report Count (2)                   */
			  0x81, 0x02,        /*     Input (Data,Var, Abs)               */
		  0xC0,    /*     END_COLLECTION	             */
          // 44 bytes
        /* USER CODE END 0 */
        0xC0 /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */
#ifdef USE_EXTRAS
__ALIGN_BEGIN static uint8_t Pedal_HID_ReportDesc[USBD_PEDAL_HID_REPORT_DESC_SIZE] __ALIGN_END =
    {
	0x05, 0x01,        /* Usage Page (Generic Desktop Ctrls)     */
	0x09, 0x04,        /* Usage (Joystick)                       */
	0xA1, 0x01,        /* Collection (Application)               */
		  0xA1, 0x00,        /*   Collection (Physical)              */
		  	  0x85, 0x02,		 	/*   ReportID (2)						 */
			  0x05, 0x01,        /*     Usage Page (Generic Desktop Ctrls) */
			  0x09, 0x32,        /*     Usage (Z)                          */
			  0x16, 0x01, 0xFC,  /*     Logical Minimum (-1023)            */
			  0x26, 0xFF, 0x03,  /*     Logical Maximum (1023)             */
			  0x75, 0x10,        /*     Report Size (16)                   */
			  0x95, 0x01,        /*     Report Count (1)                   */
			  0x81, 0x02,        /*     Input (Data,Var, Abs)               */

              0x09, 0x33,        /*     Usage (Rx)                          */
              0x09, 0x34,        /*     Usage (Ry)                          */
			  0x16, 0x00, 0x00,  /*     Logical Minimum (0)                 */
			  0x26, 0xFF, 0x07,  /*     Logical Maximum (2047)             */
			  0x75, 0x10,        /*     Report Size (16)                   */
			  0x95, 0x02,        /*     Report Count (2)                   */
		  0xC0,    /*     END_COLLECTION	             */
    0xC0,    /*     END_COLLECTION	             */
          // 42 bytes
};

__ALIGN_BEGIN static uint8_t Collective_HID_ReportDesc[USBD_COLLECTIVE_HID_REPORT_DESC_SIZE] __ALIGN_END =
    {
	0x05, 0x01,        /* Usage Page (Generic Desktop Ctrls)     */
	0x09, 0x04,        /* Usage (Joystick)                       */
    0xA1, 0x01,        /* Collection (Application)               */
		  0xA1, 0x00,        /*   Collection (Physical)              */
		  	  0x85, 0x03,		 	/*   ReportID (3)						 */
			  0x05, 0x09,        /*     Usage Page (Button)                */
			  0x19, 0x01,        /*     Usage Minimum (0x01)               */
			  0x29, 0x1A,        /*     Usage Maximum (0x1A)               */
			  0x15, 0x00,        /*     Logical Minimum (0)                */
			  0x25, 0x01,        /*     Logical Maximum (1)                */
			  0x95, 0x1A,        /*     Report Count (26)                   */
			  0x75, 0x01,        /*     Report Size (1)                    */
			  0x81, 0x02,        /*     Input (Data,Var,Abs)               */
              0x95, 0x01,        /*     Report Count (6)                   */
			  0x75, 0x06,        /*     Report Size (1)                    */
	          0x81, 0x03,        /*     Input (Cnst,Var,Abs)               */ 
			  0x05, 0x01,        /*     Usage Page (Generic Desktop Ctrls) */
			  0x09, 0x35,        /*     Usage (Rz)                          */
			  0x16, 0x00, 0x00,  /*     Logical Minimum (0)                 */
			  0x26, 0xFF, 0x07,  /*     Logical Maximum (2047)             */
			  0x75, 0x10,        /*     Report Size (16)                   */
			  0x95, 0x01,        /*     Report Count (1)                   */
			  0x81, 0x02,        /*     Input (Data,Var, Abs)               */
		  0xC0,    /*     END_COLLECTION	             */
        0xC0 /*     END_COLLECTION	             */
      // 50 bytes
};
#endif
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDevice;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID2_Init(void);
static int8_t CUSTOM_HID2_DeInit(void);
static int8_t CUSTOM_HID2_OutEvent(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID2_ItfTypeDef USBD_CustomHID2_fops = {CUSTOM_HID2_ReportDesc,
                                                  CUSTOM_HID2_Init,
                                                  CUSTOM_HID2_DeInit,
                                                  CUSTOM_HID2_OutEvent};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID2_Init(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID2_DeInit(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID2_OutEvent(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  //memcpy(buffer, state, 0x40);
  //USBD_CUSTOM_HID_SendReport(&hUsbDevice, (uint8_t *)buffer, 0x40);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDevice, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

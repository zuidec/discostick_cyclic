/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "input_filter.h"
#include "stdbool.h"
#include "w25q16jv.h"
#include "stepper.h"
#include "calibration.h"
#include "bitutils.h"
#include "uart.h"
#include "com_packet.h"
#include "special_hid_conf.h"
#include "usb_device.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDevice;
volatile uint16_t adc_buffer[NUM_ADC_CHANNEL] = {0};
bool IRQ_Calibrate_Flag;
cyclic_report_t cyclic_report;
axis_calibration_factors_t roll_calibrations = {0};
axis_calibration_factors_t pitch_calibrations = {0};
stepper_handle_t pitch_motor;
stepper_handle_t roll_motor;
input_filter_t roll_filter;
input_filter_t pitch_filter;
w25q16_handle_t flash_handle = {&hspi1, FLASH_CS_GPIO_Port, FLASH_CS_Pin};
uint8_t button_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len);
void init_steppers(void);

void init_flash(void);
void init_cyclic_input(void);
void update_cyclic_input(void);

void calibration_routine(void);
void calibrate_axis(axis_calibration_factors_t* cal, volatile uint16_t* adc_buf);
void load_calibrations(axis_calibration_factors_t* cal[], uint8_t calibration_count);
void save_calibrations(axis_calibration_factors_t* cal[], uint8_t calibration_count);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUM_ADC_CHANNEL);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  bool rs232_connected = true;
  if(rs232_connected)	{
	ENABLE_USBD_HID_CUSTOM2 = 1;
  }
  else	{
		ENABLE_USBD_HID_CUSTOM2 = 0;
  }
  MX_USB_DEVICE_Init();
  init_flash();
  init_cyclic_input();
  init_steppers();
  uart_handle_t uart4;
  uart_handle_t uart3;
  uart_init(&uart4, &huart4);
  uart_init(&uart3, &huart3);
  com_packet_t packet;
  com_packet_init(&packet, &hcrc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  uint64_t usb_timer = HAL_GetTick();
  uint64_t uart_timer = HAL_GetTick();
  uint32_t usb_interval_ms = 5;
  uint32_t packet_count =0x00;

  bool ack_received = false;
  uint32_t retx_count = 0;
#define RX_BUFFER_SIZE (128)
  uint8_t temp[RX_BUFFER_SIZE];
  uint8_t debuff[RX_BUFFER_SIZE];
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(IRQ_Calibrate_Flag){
		  calibration_routine();
	  }
	  update_cyclic_input();

	  // Check if CDR is pressed and release stepper motors if it is
	  if(CDR_BIT(cyclic_report.buttons))	{
		  stepper_disable(&pitch_motor);
		  stepper_disable(&roll_motor);
	  }
	  else	{
		  stepper_enable(&pitch_motor);
		  stepper_enable(&roll_motor);
	  }


	  if(HAL_GetTick()-usb_timer >= usb_interval_ms )	{
		  uint8_t temp_buffer[6] = {0x01, cyclic_report.buttons,cyclic_report.roll, cyclic_report.roll >> 8,cyclic_report.pitch, cyclic_report.pitch >> 8};
		  uint8_t otherbuffer[7] = {0x02,cyclic_report.pitch, cyclic_report.pitch >> 8,0xC0,0xDE,0xDE,0xAD};
		 uint8_t status =  USBD_CUSTOM_HID_SendReport(&hUsbDevice, temp_buffer, sizeof(temp_buffer));
		 if(status == USBD_FAIL || status == USBD_BUSY)	{
			 USBD_CUSTOM_HID_SendReport(&hUsbDevice, temp_buffer, sizeof(temp_buffer));
		 }
		 USBD_CUSTOM_HID2_SendReport(&hUsbDevice, otherbuffer, sizeof(otherbuffer));
		  //USBD_CUSTOM_HID_SendReport(&hUsbDevice, (uint8_t *)&cyclic_report, cyclic_report.size);
          usb_timer = HAL_GetTick();
	  }
	  if(uart_update(&uart4) == UART_RX_FAIL)	{
	  		  // handle rx failure
	  }
	  else if(uart4.unread_bytes > 0)	{
		   memset(temp,'\0', RX_BUFFER_SIZE);
		   uint32_t bytes_read =  fifo_peek_continuous(&uart4.rx_fifo, temp, uart4.unread_bytes);
		   //uint32_t bytes_read = uart_read(&uart4,temp,RX_BUFFER_SIZE);

		   packet_type_t packet_type = com_packet_parse(&packet,temp,bytes_read);
		   if(packet_type != COM_PACKET_FALSE)	{
			   switch(packet_type)	{
				   case COM_PACKET_ACK:
					   ack_received = true;
					   retx_count = 0;
					   break;
				   case COM_PACKET_NACK:
					   break;
				   case COM_PACKET_CMD:
					   break;
				   default:
					   break;
			   }
			   fifo_push_read_index(&uart4.rx_fifo, bytes_read);
		   }

	  }
	  if (ack_received){// && HAL_GetTick() - uart_timer >= 2){

		  uint32_t bytes_read = 0;
		  memset(temp,'\0', RX_BUFFER_SIZE);
		  bytes_read = (uint32_t)snprintf((char*)temp, RX_BUFFER_SIZE, "Packets sent: %lu", packet_count);
		  com_packet_create(&packet, temp, bytes_read);
		  (void)uart_write_packet(&uart4, &packet);
		  ack_received = false;
		  packet_count++;
          uart_timer = HAL_GetTick();

	  }
	  else if(ack_received == false && HAL_GetTick() - uart_timer >= 150)	{
		  if(retx_count < 3)	{

			  uint32_t bytes_read = 0;
		  	  memset(temp,'\0', RX_BUFFER_SIZE);
		  	  bytes_read = (uint32_t)snprintf((char*)temp, RX_BUFFER_SIZE, "Packets sent: %lu", packet_count);
		  	  com_packet_create(&packet, temp, bytes_read);
		  	  (void)uart_write_packet(&uart4, &packet);
		  	  retx_count++;
		  }
		  else	{
			  retx_count = 0;
			  ack_received = true;
		  }
	  }
	  else if(packet_count ==0)	{
		  uint32_t bytes_read = 0;
          memset(temp,'\0', RX_BUFFER_SIZE);
		  bytes_read = (uint32_t)snprintf((char*)temp, RX_BUFFER_SIZE, "Packets sent: %lu", packet_count);
		  packet_count++;
		  com_packet_create(&packet, temp, bytes_read);
		  (void)uart_write_packet(&uart4, &packet);
		  //(void)uart_write(&uart4,temp,bytes_read);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void init_steppers(void)	{

	  pitch_motor.en_gpio_port = PITCH_STEP_EN_GPIO_Port;
	  pitch_motor.en_gpio_pin = PITCH_STEP_EN_Pin;
	  pitch_motor.dir_gpio_port = PITCH_STEP_DIR_GPIO_Port;
	  pitch_motor.dir_gpio_pin = PITCH_STEP_DIR_Pin;
	  pitch_motor.pul_gpio_port = PITCH_STEP_PUL_GPIO_Port;
	  pitch_motor.pul_gpio_pin = PITCH_STEP_PUL_Pin;
	  pitch_motor.timer = &htim8;
	  pitch_motor.mode = MICROSTEP_16;

	  roll_motor.en_gpio_port = ROLL_STEP_EN_GPIO_Port;
	  roll_motor.en_gpio_pin = ROLL_STEP_EN_Pin;
	  roll_motor.dir_gpio_port = ROLL_STEP_DIR_GPIO_Port;
	  roll_motor.dir_gpio_pin = ROLL_STEP_DIR_Pin;
	  roll_motor.pul_gpio_port = ROLL_STEP_PUL_GPIO_Port;
	  roll_motor.pul_gpio_pin = ROLL_STEP_PUL_Pin;
	  roll_motor.timer = &htim8;
	  roll_motor.mode = MICROSTEP_16;

	  stepper_enable(&pitch_motor);
	  stepper_enable(&roll_motor);
}

void init_flash(void)	{

	  //volatile uint64_t id_value = w25q16_get_id(&flash_handle);
	  //volatile uint32_t jedec_id = w25q16_get_jedec_id(&flash_handle);

}

void init_cyclic_input(void)	{
	  axis_calibration_factors_t* axis_ptr[2] = {&pitch_calibrations, &roll_calibrations};
	cyclic_report.buttons = 0;
	  cyclic_report.size = sizeof(cyclic_report.buttons) + sizeof(cyclic_report.roll) + sizeof(cyclic_report.pitch);
	  HAL_Delay(10); // Allow extra time for flash chip to come up, probably unnecessary but is for extra safe-ness
	  load_calibrations(axis_ptr,2);
	  input_filter_init(&roll_filter, ROLL_ALPHA);
	  input_filter_init(&pitch_filter, PITCH_ALPHA);
}

void update_cyclic_input(void)	{
	  cyclic_report.buttons = 0;
	  cyclic_report.buttons = (HAL_GPIO_ReadPin(CYC_AP_REL_GPIO_Port, CYC_AP_REL_Pin)) |
			  	  	  	  	  (HAL_GPIO_ReadPin(CYC_ICS_SW_GPIO_Port, CYC_ICS_SW_Pin) << 1) |
							  (HAL_GPIO_ReadPin(CYC_CARGO_REL_GPIO_Port, CYC_CARGO_REL_Pin) << 2) |
							  (HAL_GPIO_ReadPin(CYC_HAT_UP_GPIO_Port, CYC_HAT_UP_Pin) << 3) |
							  (HAL_GPIO_ReadPin(CYC_HAT_DOWN_GPIO_Port, CYC_HAT_DOWN_Pin) << 4) |
							  (HAL_GPIO_ReadPin(CYC_HAT_RIGHT_GPIO_Port, CYC_HAT_RIGHT_Pin) << 5) |
							  (HAL_GPIO_ReadPin(CYC_HAT_LEFT_GPIO_Port, CYC_HAT_LEFT_Pin) << 6) |
							  (HAL_GPIO_ReadPin(CYC_CDR_REL_GPIO_Port, CYC_CDR_REL_Pin) << 7);

	  cyclic_report.buttons = (cyclic_report.buttons ^ CYCLIC_BUTTON_MASK);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)	{
	cyclic_report.roll = input_filter_update(&roll_filter, (float) apply_calibration(&roll_calibrations,adc_buffer[0]));
	cyclic_report.pitch = input_filter_update(&pitch_filter, (float) apply_calibration(&pitch_calibrations, adc_buffer[1]));
}

void calibration_routine(void)	{
	  axis_calibration_factors_t* axis_ptr[2] = {&pitch_calibrations, &roll_calibrations};
	  calibrate_axis(&roll_calibrations, &adc_buffer[0]);
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	  calibrate_axis(&pitch_calibrations, &adc_buffer[1]);
	  uint64_t time = HAL_GetTick();
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
	  while(HAL_GetTick() - time < 100){;;}
	  time = HAL_GetTick();
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
	  while(HAL_GetTick() - time < 100){;;}
	  time = HAL_GetTick();
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
	  while(HAL_GetTick() - time < 100){;;}
	  time = HAL_GetTick();
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

	  save_calibrations(axis_ptr, 2);
	  IRQ_Calibrate_Flag = false;

}

void calibrate_axis(axis_calibration_factors_t* cal, volatile uint16_t* adc_buf)	{

	clear_calibration(cal);

	uint64_t time = HAL_GetTick();
	while(HAL_GetTick()-time < 250)	{;;}
	while(HAL_GPIO_ReadPin(SW_OK_GPIO_Port, SW_OK_Pin))	{
		if(HAL_GetTick() - time > 500 ){
			HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
			time = HAL_GetTick();
		}
	}

	int16_t val1 = (int16_t)adc_buf[0];
	while(HAL_GetTick()-time < 50)	{;;}
	int16_t val2 = (int16_t)adc_buf[0];
	while(HAL_GetTick()-time < 50)	{;;}
	int16_t val3 = (int16_t)adc_buf[0];

	cal->physical_max = (val1+val2+val3)/3;
	time = HAL_GetTick();
	while(HAL_GetTick()-time < 250)	{;;}
	while(HAL_GPIO_ReadPin(SW_OK_GPIO_Port, SW_OK_Pin))	{
			if(HAL_GetTick() - time > 150 ){
				HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
				time = HAL_GetTick();
			}
		}

	val1 = (int16_t)adc_buf[0];
	while(HAL_GetTick()-time < 50)	{;;}
	val2 = (int16_t)adc_buf[0];
	while(HAL_GetTick()-time < 50)	{;;}
	val3 = (int16_t)adc_buf[0];

	cal->physical_min = (val1+val2+val3)/3;

	recalculate_calibration(cal);
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
}

void load_calibrations(axis_calibration_factors_t* cal[], uint8_t calibration_count)	{
	uint8_t buffer_size = 0;
	for(uint8_t i=0; i < calibration_count; i++)	{
		buffer_size += CALIBRATION_FACTOR_SIZE;
	}
	uint8_t size_offset = buffer_size / calibration_count;
	uint8_t data_buffer[buffer_size];

	w25q16_read(&flash_handle, FLASH_CALIBRATION_ADDRESS, data_buffer, buffer_size);
	for(uint8_t i=0; i< calibration_count; i++){
		set_calibration(cal[i], &data_buffer[i*size_offset], size_offset);
	}
}

void save_calibrations(axis_calibration_factors_t* cal[], uint8_t calibration_count)	{
	uint8_t buffer_size = 0;
	for(uint8_t i=0; i < calibration_count; i++)	{
		buffer_size += CALIBRATION_FACTOR_SIZE;
	}
	uint8_t size_offset = buffer_size / calibration_count;
	uint8_t data_buffer[buffer_size];

	for(uint8_t i=0; i< calibration_count; i++){
		get_calibration(cal[i], &data_buffer[i*size_offset], size_offset);
	}

	w25q16_sector_erase_4k(&flash_handle, FLASH_CALIBRATION_ADDRESS);
	uint8_t busy_flag = w25q16_read_SR1(&flash_handle)&0x01;
	uint64_t timer = HAL_GetTick();
	while(busy_flag && HAL_GetTick()-timer<200)	{
		busy_flag = w25q16_read_SR1(&flash_handle)&0x01;
	}
	w25q16_write(&flash_handle, FLASH_CALIBRATION_ADDRESS, data_buffer, buffer_size);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	{
	IRQ_Calibrate_Flag = true;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

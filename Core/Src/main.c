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
#include "usb_device.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
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
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_UART4_Init(void);
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
  MX_USB_DEVICE_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUM_ADC_CHANNEL);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
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
  uint32_t usb_interval_ms = 10;
  uint32_t packet_count =0;

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
	  uint8_t temp_buffer[5] = {cyclic_report.buttons,cyclic_report.roll, cyclic_report.roll >> 8,cyclic_report.pitch, cyclic_report.pitch >> 8};
	  if(HAL_GetTick()-usb_timer >= usb_interval_ms )	{
		  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, temp_buffer, sizeof(temp_buffer));
		  //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *)&cyclic_report, cyclic_report.size);
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim8.Init.Period = 2-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim8, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_1_Pin|LED_2_Pin|LED_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PITCH_STEP_DIR_Pin|PITCH_STEP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ROLL_STEP_DIR_Pin|ROLL_STEP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_OK_Pin */
  GPIO_InitStruct.Pin = SW_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_OK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_CAL_Pin */
  GPIO_InitStruct.Pin = SW_CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW_CAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_4_Pin */
  GPIO_InitStruct.Pin = LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UART4_EN_Pin */
  GPIO_InitStruct.Pin = UART4_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART4_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UART2_EN_Pin */
  GPIO_InitStruct.Pin = UART2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART2_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CYC_ICS_SW_Pin CYC_CARGO_REL_Pin CYC_CDR_REL_Pin CYC_AP_REL_Pin
                           CYC_HAT_LEFT_Pin CYC_HAT_DOWN_Pin CYC_HAT_UP_Pin */
  GPIO_InitStruct.Pin = CYC_ICS_SW_Pin|CYC_CARGO_REL_Pin|CYC_CDR_REL_Pin|CYC_AP_REL_Pin
                          |CYC_HAT_LEFT_Pin|CYC_HAT_DOWN_Pin|CYC_HAT_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : CYC_HAT_RIGHT_Pin */
  GPIO_InitStruct.Pin = CYC_HAT_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CYC_HAT_RIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PITCH_STEP_DIR_Pin PITCH_STEP_EN_Pin */
  GPIO_InitStruct.Pin = PITCH_STEP_DIR_Pin|PITCH_STEP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ROLL_STEP_DIR_Pin ROLL_STEP_EN_Pin */
  GPIO_InitStruct.Pin = ROLL_STEP_DIR_Pin|ROLL_STEP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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

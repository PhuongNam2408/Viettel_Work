/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include <string.h>
#include <stdio.h>
#include "keypad.h"
#include "flash.h"
#include "ds18b20.h"
#include "DC_motor.h"
#include "CLCD_I2C.h"
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
I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

osThreadId Display_TaskHandle;
osThreadId Check_Key_TaskHandle;
osThreadId Nhietdo_TaskHandle;
osThreadId SD_Card_TaskHandle;
osThreadId DC_Motor_TaskHandle;
osThreadId Flash_Mail_TaskHandle;
osThreadId Flash_Msg_TaskHandle;
osMailQId Mail_QueueHandle;
osMessageQId Msg_QueueHandle;
osSemaphoreId DC_Motor_SemHandle;
/* USER CODE BEGIN PV */
FATFS fs;						//FATFS file system 
FIL fil;						//variable for file 
FRESULT fresult;		//state of action to file

UINT br, bw;				//number of bytes for file read/write , use in f_read or f_write func

uint32_t temp_min = 10;		//Nguong nhiet do
uint32_t temp_max = 30;		//Nguong nhiet do
uint32_t Ds18b20_temp;		//Gia tri nhiet do thuc te

char tempVal[3];		//Bien trung gian dua gia tri nhiet do ve String
char speedVal[4];	//Bien trung gian dua gia tri toc do ve String

uint8_t mode=5;			//Lua chon che do dieu khien dong co
uint8_t on=5;								//variable for on/off motor

uint32_t current_speed = 0;		//Toc do duoc dat vao dong co

CLCD_I2C_Name LCD1;			//LCD handler

uint8_t keypad_speed = 5;		//%speed value from keypad

uint32_t count=5;						//Bien dem so lan viet vao the SD

uint32_t page_127_addr = FLASH_BASE + 127*1024;			//Dia chi cua PAGE 127, moi page la 1kB = 1*2^10 Byte
uint32_t page_126_addr = FLASH_BASE + 126*1024;			//Dia chi cua PAGE 126, moi page la 1kB = 1*2^10 Byte

uint8_t key = 0xFF;		//Luu tru nut vua duoc nhan

char *file_name = "viettel.txt";			//Tên file trong SD

//Structure to save in mail flash
typedef struct 
{
	uint8_t on_t;
	uint8_t mode_t;
	uint8_t keypad_speed_t;
}
Data_Flash_t;

char buffer[1024];	//buffer to send data to UART

volatile uint16_t Timer1, Timer2;		//Bien timer xu ly Low level cua FATFS
volatile uint8_t FatFs_Cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Start_Display_Task(void const * argument);
void Start_Check_Key_Task(void const * argument);
void Start_Nhietdo_Task(void const * argument);
void Start_SD_Card_Task(void const * argument);
void Start_DC_Motor_Task(void const * argument);
void Start_Flash_Mail_Task(void const * argument);
void Start_Flash_Msg_Task(void const * argument);

/* USER CODE BEGIN PFP */
int bufsize(char *buf);						
void send_uart(char *string);
void bufclear (void);
void get_SD_size(void);
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_IWDG_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);		//Init PWM on timer 3, channel 1
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); 	//Init PWM on timer 3, channel 2
	DC_motor_Enable(DC_MOTOR_ENABLE);						//ENABLE dc_motor
	CLCD_I2C_Init(&LCD1,&hi2c1,0x4e,20,4);			//Init LCD with i2c 1
	
	/*Khoi dong*/
	HAL_IWDG_Refresh(&hiwdg);			//Load IWDG counter
	//check iwdg reset flag - check reference manual
	if(((RCC->CSR >> 29) & 1U) == 1)	
	{
		sprintf(buffer, "\nChuong trinh da duoc reset\n");	//save data want to display to buffer
		send_uart(buffer);
		bufclear();		//Clear buffer
		
		RCC->CSR |= 1 << 24;			//remove reset flag
	}
	
	
	/*Doc bo nho flash tai page 126, gia tri count: 4 byte, voi byte o dia chi cao la MSB */
	count = FLASH_ReadWorld(page_126_addr);			//Read 4 byte in page 126
	
	/*Doc bo nho flash tai page 127, 3 byte dau` la byte on, mode, keypad_speed
		byte thu 4 la byte trông', 4 byte tiep theo la gia tri count, voi byte o 
		dia chi cao la MSB */
	on = FLASH_Read1Byte(page_127_addr);
	mode = FLASH_Read1Byte(page_127_addr + 1);
	keypad_speed = FLASH_Read1Byte(page_127_addr + 2);
	

	
	//So sanh cac gia tri doc duoc co bang` 0xFF hay khong, neu co thi dua cac gia tri ve 0
	//NOTE: 0xFF la gia tri default cua flash, tuc la flash chua bao gio duoc ghi gia tri
	on = (on == 0xFF) ? 0 : on;			
	mode = (mode == 0xFF) ? 0 : mode;
	keypad_speed = (keypad_speed == 0xFF) ? 0 : keypad_speed;
	count = (count == 0xFFFFFFFF) ? 0 : count;
	

	/*Hien thi ban dau cua chuong trinh*/
	sprintf(buffer, "\nChuong trinh bat dau\n");	//save data want to display to buffer
	send_uart(buffer);
	bufclear();		//Clear buffer
	
	fresult = f_mount(&fs, "/", 1);		
	if(fresult != FR_OK)
	{
		//Error handler
		sprintf(buffer, "\nLoi tai Khoi dau - Mount, %d\n", fresult);
		send_uart(buffer);
		bufclear();
		Error_Handler();
	}
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of DC_Motor_Sem */
  osSemaphoreDef(DC_Motor_Sem);
  DC_Motor_SemHandle = osSemaphoreCreate(osSemaphore(DC_Motor_Sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Mail_Queue */
  osMailQDef(Mail_Queue, 16, Data_Flash_t);
  Mail_QueueHandle = osMailCreate(osMailQ(Mail_Queue), NULL);

  /* definition and creation of Msg_Queue */
  osMessageQDef(Msg_Queue, 16, uint32_t);
  Msg_QueueHandle = osMessageCreate(osMessageQ(Msg_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Display_Task */
  osThreadDef(Display_Task, Start_Display_Task, osPriorityLow, 0, 128);
  Display_TaskHandle = osThreadCreate(osThread(Display_Task), NULL);

  /* definition and creation of Check_Key_Task */
  osThreadDef(Check_Key_Task, Start_Check_Key_Task, osPriorityHigh, 0, 128);
  Check_Key_TaskHandle = osThreadCreate(osThread(Check_Key_Task), NULL);

  /* definition and creation of Nhietdo_Task */
  osThreadDef(Nhietdo_Task, Start_Nhietdo_Task, osPriorityRealtime, 0, 170);
  Nhietdo_TaskHandle = osThreadCreate(osThread(Nhietdo_Task), NULL);

  /* definition and creation of SD_Card_Task */
  osThreadDef(SD_Card_Task, Start_SD_Card_Task, osPriorityNormal, 0, 128);
  SD_Card_TaskHandle = osThreadCreate(osThread(SD_Card_Task), NULL);

  /* definition and creation of DC_Motor_Task */
  osThreadDef(DC_Motor_Task, Start_DC_Motor_Task, osPriorityHigh, 0, 128);
  DC_Motor_TaskHandle = osThreadCreate(osThread(DC_Motor_Task), NULL);

  /* definition and creation of Flash_Mail_Task */
  osThreadDef(Flash_Mail_Task, Start_Flash_Mail_Task, osPriorityNormal, 0, 128);
  Flash_Mail_TaskHandle = osThreadCreate(osThread(Flash_Mail_Task), NULL);

  /* definition and creation of Flash_Msg_Task */
  osThreadDef(Flash_Msg_Task, Start_Flash_Msg_Task, osPriorityNormal, 0, 128);
  Flash_Msg_TaskHandle = osThreadCreate(osThread(Flash_Msg_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL15;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1874;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 60-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 60-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 60000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Xu ly thoi gian cua Low level FATFS
void SD_Timer_Handler(void)
{
	if(Timer1 > 0)
	{
		Timer1--;
	}
	
	if(Timer2 > 0)
	{
		Timer2--;
	}
}

/**
  * @brief GPIO Output Configuration
  * @retval None
  */
void GPIO_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	//Khai bao bien GPIO_Init theo kieu du lieu GPIO_InitTypeDef
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//Chon chan
	GPIO_InitStruct.Pin = GPIO_Pin;
	//Chon che do Output PushPull
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	//Initialize 
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief GPIO Input Configuration
  * @retval None
  */
void GPIO_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	//Khai bao bien GPIO_Init theo kieu du lieu GPIO_InitTypeDef
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	//Chon chan
	GPIO_InitStruct.Pin = GPIO_Pin;
	//Chon che do Input
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	//Khong keo tro
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	//Initialize 
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief Delay microsecond using timer 2
	* @variable time delay in us
  * @retval None
  */
void delay_us(uint16_t time_us)
{
//	HAL_TIM_Base_Start(&htim2); 		
	TIM2->CR1|=(TIM_CR1_CEN);			//Bat timer2 len
	__HAL_TIM_SET_COUNTER(&htim2,0);		//dat gia tri counter cua timer = 0
	while(__HAL_TIM_GET_COUNTER(&htim2)<time_us);			
}

/**
  * @brief Set %speed of PWM output
	* @variable tim_channel: channel want to change speed
	* @variable speed: speed in % 
  * @retval None
  */
void PWM_Speed(uint32_t tim_channel, uint32_t speed){
	uint32_t max_count = htim3.Init.Period; 	//So lan dem nhieu nhat co the cua timer
	speed = (speed > 100)	? 100 : speed ;
	speed = speed*max_count/100;		//Do speed la % nen can dat gia tri so lan dem
	TIM3->CR1 |= 1 << 0;
	__HAL_TIM_SET_COMPARE(&htim3,	tim_channel, speed);
}

/**
  * @brief Convert temparature and speed to string in order to Display
  * @retval None
  */
void Convert_data(void){
	sprintf(tempVal, "%2.0f", (float)Ds18b20_temp);
	sprintf(speedVal, "%3.0f", (float)current_speed);
}

void UART_Display(void){
	//Dong 1
	sprintf(buffer, "\nNhiet do: %2.0fºC\n", (float)Ds18b20_temp);
	send_uart(buffer);
	bufclear();
	//Dong 2
	sprintf(buffer, "Toc do: %3.0f%%\n", (float)current_speed);
	send_uart(buffer);
	bufclear();
	//Dong 3
	sprintf(buffer, "Size of file: %ld\n", f_size(&fil));
	send_uart(buffer);
	bufclear();	
}

/**
  * @brief Dipslay Temp and % Speed at the moment use I2C_LCD 
  * @retval None
  */
void LCD_Display(void){
	//Line 1
	CLCD_I2C_SetCursor(&LCD1,0,0);
	CLCD_I2C_WriteString(&LCD1,"Nhiet do:");
	CLCD_I2C_SetCursor(&LCD1,9,0);
	CLCD_I2C_WriteString(&LCD1,tempVal);
	CLCD_I2C_SetCursor(&LCD1,11,0);
	CLCD_I2C_WriteChar(&LCD1,0xDF);	//Ký tu "º"
	CLCD_I2C_SetCursor(&LCD1,12,0);
	CLCD_I2C_WriteString(&LCD1,"C");	
	//Line 2
	CLCD_I2C_SetCursor(&LCD1,0,1);
	CLCD_I2C_WriteString(&LCD1,"Toc do:");
	CLCD_I2C_SetCursor(&LCD1,7,1);
	CLCD_I2C_WriteString(&LCD1,speedVal);
	CLCD_I2C_SetCursor(&LCD1,10,1);
	CLCD_I2C_WriteString(&LCD1,"%");
}

/**
  * @brief Send the string use UART1
  * @param *string: address of first character
  * @retval None
  */
void send_uart(char *string)
{
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1, (uint8_t *)string, len, HAL_MAX_DELAY);
}

/**
  * @brief Find the size of buffer
  * @param pointer to buffer wanna find the size
  * @retval size of buffer
  */
int bufsize(char *buf)
{
	int i=0;
	while(*buf++ != '\0')
	{
		i++;
	}
	return i;
}

/**
  * @brief Clear the buffer variable
  * @param None
  * @retval None
  */
void bufclear (void)
{
	for(int i=0; i < 1024; i++)
	{
		buffer[i] = '\0';
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_Display_Task */
/**
  * @brief  Function implementing the Display_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Display_Task */
void Start_Display_Task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		HAL_IWDG_Refresh(&hiwdg);		//Reload IWDG counter
		//Chuyen doi du lieu ve string 
		Convert_data();
		
		LCD_Display();
		
		UART_Display();
		
    osDelay(200);			//Cycle 200ms
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Check_Key_Task */
/**
* @brief Function implementing the Check_Key_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Check_Key_Task */
void Start_Check_Key_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Check_Key_Task */
	Data_Flash_t *data_temp_t;		//Bien structure de gui lieu
	uint8_t flag = 0;		//Bien kiem tra xem co nut nhan hay khong
  /* Infinite loop */
  for(;;)
  {
		HAL_IWDG_Refresh(&hiwdg);		//Reload IWDG counter

		key = keypad_read();
		if(key == 1){
			//Key 1: on/off dong co
			on = on % 2 + 1;			//Cap nhat gia tri on	
			flag = 1;
		}
		if(key == 2){
			//Key 2: thay doi mode dieu khien toc do
			mode = mode % 2 + 1;
			flag = 1;
		}
		if(key == 3){
			//Key 3: tang toc do dong co	
			if(keypad_speed >= 100)
			{
				keypad_speed = 100;
			}
			else 
			{
				keypad_speed = keypad_speed + 5;	
			}
			flag = 1;
		}
		if(key == 4){
			//Key 4: giam toc do dong co
			if(keypad_speed <= 0)
			{
				keypad_speed = 0;
			}
			else 
			{
				keypad_speed = keypad_speed - 5;
			}
			flag = 1;
		}	
		if(flag == 1)
		{
			/*Release Semaphore*/
			osSemaphoreRelease(DC_Motor_SemHandle);
			
			/*Send Mail Queue*/
			//Cap phat bo nho
			data_temp_t = osMailAlloc(Mail_QueueHandle,osWaitForever);
			//Luu gia tri
			data_temp_t->on_t = on;
			data_temp_t->mode_t = mode;
			data_temp_t->keypad_speed_t = keypad_speed;
			//Gui vao mail queue
			osMailPut(Mail_QueueHandle, data_temp_t);
			
			flag = 0;
		}		
    osDelay(5);
  }
  /* USER CODE END Start_Check_Key_Task */
}

/* USER CODE BEGIN Header_Start_Nhietdo_Task */
/**
* @brief Function implementing the Nhietdo_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Nhietdo_Task */
void Start_Nhietdo_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Nhietdo_Task */
  /* Infinite loop */
  for(;;)
  {
		Ds18b20_temp = Ds18b20_NhietDo();
		if (Ds18b20_temp == 1)
		{
			/*Cam bien bi loi*/
			sprintf(buffer, "\nLoi cam bien nhiet do\n");
			send_uart(buffer);		//Hien thi UART cam bien loi
			bufclear();
			Error_Handler();		//Vong lap vo han
		}
		/*Release Semaphore*/
		osSemaphoreRelease(DC_Motor_SemHandle); 
		osDelay(5000);	//cycle 5s
  }
  /* USER CODE END Start_Nhietdo_Task */
}

/* USER CODE BEGIN Header_Start_SD_Card_Task */
/**
* @brief Function implementing the SD_Card_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SD_Card_Task */
void Start_SD_Card_Task(void const * argument)
{
  /* USER CODE BEGIN Start_SD_Card_Task */
  /* Infinite loop */
  for(;;)
  {
		/*Action with SD Card*/
		
		//Open file with OPEN_ALWAYS and WRITE mode
		fresult = f_open(&fil, file_name, FA_OPEN_ALWAYS|FA_WRITE);		
		if(fresult != FR_OK){
			//Error handler
			sprintf(buffer, "\nLoi tai SD task - Open, %d\n", fresult);
			send_uart(buffer);
			bufclear();
			Error_Handler();
		}
		
		//Dua con tro cua file xuong phia cuoi de update file
		fresult = f_lseek(&fil, f_size(&fil)); 	
		if(fresult != FR_OK){
			//Error handler
			sprintf(buffer, "\nLoi tai SD task - Lseek, %d\n", fresult);
			send_uart(buffer);
			bufclear();
			Error_Handler();
		}		
		
		//Write data to file
		sprintf(buffer, "%d) Nhiet do: %2.0fºC; Toc do: %3.0f%%\n",count++, (float)Ds18b20_temp, (float)current_speed);
		fresult = f_write(&fil, buffer, bufsize(buffer),&bw);
		if(fresult != FR_OK){
			//Error handler
			sprintf(buffer, "\nLoi tai SD task - Write, %d\n", fresult);
			send_uart(buffer);
			bufclear();
			Error_Handler();
		}				
		bufclear();			//Clear buffer after write finish
		
		//Close file
		fresult = f_close(&fil);
		if(fresult != FR_OK){
			//Error handler
			sprintf(buffer, "\nLoi tai SD task - Close, %d\n", fresult);
			send_uart(buffer);
			bufclear();
			Error_Handler();
		}						
		
		/*Dua gia tri count vao Msg Queue*/
		osMessagePut(Msg_QueueHandle, count, osWaitForever);
		
		
    osDelay(10000);		//Cycle 10s
  }
  /* USER CODE END Start_SD_Card_Task */
}

/* USER CODE BEGIN Header_Start_DC_Motor_Task */
/**
* @brief Function implementing the DC_Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DC_Motor_Task */
void Start_DC_Motor_Task(void const * argument)
{
  /* USER CODE BEGIN Start_DC_Motor_Task */
  /* Infinite loop */
  for(;;)
  {
		/*Wait Semaphore*/
		osSemaphoreWait(DC_Motor_SemHandle,osWaitForever);
		
		/*check motor state*/
		if(on % 2 == 0)
		{
			//Dong co dang tat => toc do = 0
			current_speed = 0;
		}
		else if(mode % 2 == 0)
		{
			//Che do dieu khien theo nut nhan 
			current_speed = keypad_speed;
		}
		else 
		{
			//Che do dieu khien theo nhiet do
			current_speed = DC_motor_Speed_Based_on_Temp(Ds18b20_temp);
		}
		
		DC_motor_PWM(current_speed);		//export speed to DC_motor
		osDelay(1);
  }
  /* USER CODE END Start_DC_Motor_Task */
}

/* USER CODE BEGIN Header_Start_Flash_Mail_Task */
/**
* @brief Function implementing the Flash_Mail_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Flash_Mail_Task */
void Start_Flash_Mail_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Flash_Mail_Task */
	Data_Flash_t *value_t;		//Bien luu tru Mail Queue nhan duoc
	osEvent ret_mail;			//Bien luu tru Event cua Mail Queue
	uint8_t temp[4];			//Bien trung gian de luu vao flash
  /* Infinite loop */
  for(;;)
  {
		/*Wait Mail Queue*/
		ret_mail = osMailGet(Mail_QueueHandle, osWaitForever);
		 
		value_t = ret_mail.value.p;
		
		temp[0] = value_t->on_t;
		temp[1] = value_t->mode_t;
		temp[2] = value_t->keypad_speed_t;
		temp[3] = 0;	//Dummy variable - Do ta chi co the viet 2 byte cung luc vao flash
		
		//Truoc khi viet vao flash => Can clear truoc
		FLASH_ErasePage(page_127_addr);							
		FLASH_WriteArray(page_127_addr, temp, 4);		//Viet vao page 127
		
		
    osDelay(10);
  }
  /* USER CODE END Start_Flash_Mail_Task */
}

/* USER CODE BEGIN Header_Start_Flash_Msg_Task */
/**
* @brief Function implementing the Flash_Msg_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Flash_Msg_Task */
void Start_Flash_Msg_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Flash_Msg_Task */
	uint32_t temp;			//Bien dem nhan gia tri Msg Queue
	osEvent ret_msg;		//Event nhan duoc tu Msg Queue
  /* Infinite loop */
  for(;;)
  {
		//Nhan event 
		ret_msg = osMessageGet(Msg_QueueHandle, osWaitForever);

		//Luu gia tri va viet vao Page 126 cua flash
		temp = ret_msg.value.v;
		FLASH_ErasePage(page_126_addr);			//Xoa page
		FLASH_WriteWord(page_126_addr, temp);
		
    osDelay(10);
  }
  /* USER CODE END Start_Flash_Msg_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	FatFs_Cnt++;
	if(FatFs_Cnt >= 1)
	{
		FatFs_Cnt = 0;
		SD_Timer_Handler();
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
	send_uart("\nLOI!!!!!!!!!!!!!!!");
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

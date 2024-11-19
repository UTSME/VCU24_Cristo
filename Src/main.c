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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _PIN {
	GPIO_TypeDef* port;
	uint16_t pin;
} PIN;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

// CANBUS
const uint32_t CANID_PDMMESSAGE0 = 0x520;
const uint32_t CANID_PDMMESSAGE1 = 0x521;
const uint32_t CANID_PDMMESSAGE2 = 0x522;
const uint32_t CANID_PDMMESSAGE3 = 0x524;

const uint32_t ADC_RESOLUTION = 12;
const float SENSOR_MIN = 0.1;
const float SENSOR_MAX = 0.9;

bool print_throttle = true;
bool print_brake = false;
bool print_canbus = true;

enum CarStates
{
  IDLE,
  PRECHARGED,
  RTD,
  FAULT
} carState;


// Start Button
const PIN START_BUTTON_PIN = {GPIOB, GPIO_PIN_9};
uint8_t start_button_press = false;

// Fault LEDs
struct Faults
{
	uint8_t ams;
	uint8_t pdoc;
	uint8_t imd;
	uint8_t bspd;
} faults;

const PIN AMS_FAULT_LED = {GPIOC, GPIO_PIN_14};
const PIN IMD_FAULT_LED = {GPIOF, GPIO_PIN_9};
const PIN PDOC_FAULT_LED = {GPIOC, GPIO_PIN_15};
const PIN BSPD_FAULT_LED = {GPIOE, GPIO_PIN_6};


// PDM State
struct PDM_Status
{
	uint8_t precharge;
	uint8_t precharged;
	uint8_t rtd;
	uint8_t drive_enable;
} pdm_status;


// ADC Raw
struct ADC_Raw
{
  uint32_t tps1;
  uint32_t tps2;
  uint32_t front;
  uint32_t rear;
} adc_raw;



// Throttle Configuration


//struct ADC_Throttle
//{
//  uint32_t tps1;
//  uint32_t tps2;
//} adc_throttle;

const uint32_t TPS1_MIN = 100;
const uint32_t TPS1_MAX= 1000;

const uint32_t TPS2_MIN = 200;
const uint32_t TPS2_MAX= 1500;

const uint32_t TORQUE_MIN = 0;
const uint32_t TORQUE_MAX = 30000;


// Brake Configuration


//struct ADC_Brake
//{
//  uint32_t front;
//  uint32_t rear;
//} adc_brake;

const uint32_t BRAKE_FRONT_MIN = 100;
const uint32_t BRAKE_FRONT_MAX = 1100;

const uint32_t BRAKE_REAR_MIN = 200;
const uint32_t BRAKE_REAR_MAX = 1200;

const uint32_t BRAKE_PRESSURE_MIN = 0.0;
const uint32_t BRAKE_PRESSURE_MAX = 13789;

const uint32_t BRAKE_PRESSURE_THRESHOLD = 3000; // For checkAppsBrakePlausibilityError()

// CAN message frequency
const uint32_t TORQUE_SEND_FREQ = 5000;
const uint32_t FAULT_CHECK_FREQ = 5000;


// Mapped values
int mc_torque = 0;
int throttle_pos = 0;
int brake_avg = 0;


//uint16_t ADC_Throttle[2];
//uint16_t ADC_Brake[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
void readCAN(void);
int getTorqueCommand(void);
void sendTorqueCommand(void);
void sendBrakePressure(void);
void sendStartSw(void);
void updateLEDs(void);
bool checkAppsBrakePlausibilityError(void);
bool checkFault(void);



float constrain(float val, float lower, float upper);
float map(float val, float fromLow, float fromHigh, float toLow, float toHigh);


void print_CANBUS_RxData(void);

int _write(int file, char *ptr, int len){

	for(int i=0; i < len; i++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}

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
  carState = IDLE;
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
  MX_ADC3_Init();
  MX_CAN_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc3, &adc_raw, 4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	HAL_Delay(1000);


	TxData[0] = 0xff;

//	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
//	{
//	  /* Transmission request Error */
//	  Error_Handler();
//	}
	HAL_TIM_Base_Start(&htim7);

	switch (carState)
	{
		// Idle
		case IDLE:
			updateLEDs();
			// NextState = Fault
			if(checkFault()) {
				carState = FAULT;
				printf("\tNext State: FAULT\n");
			}

			// If precharge complete
			else if (pdm_status.precharged) {
				// NextState = Precharged
				carState = PRECHARGED;
				printf("\tNext State: PRECHARGED\n");
			}
			break;


		// Precharged
		case PRECHARGED:

			updateLEDs();
			// NextState = Fault
			if(checkFault()) {
				carState = FAULT;
				printf("\tNext State: FAULT\n");
			}

			// Poll Start Button
			else if (!HAL_GPIO_ReadPin(START_BUTTON_PIN.port, START_BUTTON_PIN.pin))
			{
				sendStartSw();
				// NextState = RTD
				carState = RTD;
				printf("\tNext State: RTD\n");
			}
			break;


		// Ready To Drive
		case RTD:
			// NextState = Fault
			if(checkFault()) {
				carState = FAULT;
				printf("\tNext State: FAULT\n");
			}

			// Send Torque to MC
			else{
				sendTorqueCommand();
				sendBrakePressure();
			}
			break;


		// Fault
		case FAULT:
			updateLEDs();

			break;

	}

//	printf("Hello World\n");

//  HAL_ADC_Start_DMA(&hadc3, AD_RES_BUFFER, 2);
//  printf("TPS1: %d, TPS2: %d", adc_throttle.tps1, adc_throttle.tps2);
//  printf("TPS1: %d, TPS2: %d", adc_throttle, adc_throttle);
//  HAL_Delay(1);
//  HAL_ADC_Stop_DMA(&hadc3);
////		hsdadc1.Instance->CR2 |= SDADC_CR2_RSWSTART;
////		HAL_SDADC_PollForConversion(&hsdadc1, 1000);
////	int rawValue = HAL_ADC_GetValue(&hadc3);
//	printf("CH2: %d\n", AD_RES_BUFFER[0]);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  /* ADC1 DMA Init */
//  hdma_adc1.Instance = DMA2_Stream0;
//  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
//  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//  hdma_adc1.Init.Mode = DMA_NORMAL;
//  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
//  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK);
//
//  __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  uint32_t filter_id = 0x00000000;
  uint32_t filter_mask = 0x1FFFFFF8;

   /* Configure the CAN Filter */
   sFilterConfig.FilterBank = 0;
   sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
   sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
   sFilterConfig.FilterIdHigh = 0x0000;
   sFilterConfig.FilterIdLow = 0x0000;
   sFilterConfig.FilterMaskIdHigh = 0x0000;
   sFilterConfig.FilterMaskIdLow = 0x0000;
   sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
   sFilterConfig.FilterActivation = ENABLE;
   sFilterConfig.SlaveStartFilterBank = 0;

//   sFilterConfig.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
//   sFilterConfig.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
//   sFilterConfig.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
//   sFilterConfig.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;

   if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
   {
     /* Filter configuration Error */
     Error_Handler();
   }

   /* Start the CAN peripheral */
   if (HAL_CAN_Start(&hcan) != HAL_OK)
   {
     /* Start Error */
     Error_Handler();
   }

   /* Activate CAN RX notification */
   if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
     /* Notification Error */
     Error_Handler();
   }


//  TxHeader.TransmitGlobalTime = DISABLE;
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Button_Pin */
  GPIO_InitStruct.Pin = Start_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Button_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sendTorqueCommand(void)
{
	// Calculate Torque Command
	uint32_t tps1 = adc_raw.tps1;
	uint32_t tps2 = adc_raw.tps2;
	uint32_t throttle_position = 0;
	uint32_t torque = 0;

	tps1 = constrain(tps1, TPS1_MIN, TPS1_MAX);
	tps1 = map(tps1, TPS1_MIN, TPS1_MAX, 0, 100);

	tps2 = constrain(tps2, TPS1_MIN, TPS1_MAX);
	tps2 = map(tps2, TPS2_MIN, TPS2_MAX, 0, 100);

	throttle_position = (tps1 - tps2) / 2;

	torque = map(throttle_position, 0, 100, TORQUE_MIN, TORQUE_MAX);


	// Transmit Torque
	TxHeader.StdId = 0x321;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 2;
	TxData [0] = torque;

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	  /* Transmission request Error */
	  Error_Handler();
	}

	return torque;
}


void sendBrakePressure(void)
{

}


void sendStartSw(void)
{

}

void updateLEDs(void)
{
	if (faults.ams) {
		HAL_GPIO_WritePin(AMS_FAULT_LED.port, AMS_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(AMS_FAULT_LED.port, AMS_FAULT_LED.pin, GPIO_PIN_RESET);
	}

	if (faults.imd) {
		HAL_GPIO_WritePin(IMD_FAULT_LED.port, IMD_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(IMD_FAULT_LED.port, IMD_FAULT_LED.pin, GPIO_PIN_RESET);
	}

	if (faults.pdoc) {
		HAL_GPIO_WritePin(PDOC_FAULT_LED.port, PDOC_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(PDOC_FAULT_LED.port, PDOC_FAULT_LED.pin, GPIO_PIN_RESET);
	}

	if (faults.bspd) {
		HAL_GPIO_WritePin(BSPD_FAULT_LED.port, BSPD_FAULT_LED.pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(BSPD_FAULT_LED.port, BSPD_FAULT_LED.pin, GPIO_PIN_RESET);
	}
}


bool checkAppsBrakePlausibilityError(void)
{
	bool appsBrakePlausibilityError = true;
	bool throttleSensorError = false;
	bool brakeSensorError = false;
	bool appsError = false;
	bool appsBrakeError = false;

	// MAP ADC VALUES
	uint32_t tps1 = adc_raw.tps1;
	uint32_t tps2 = adc_raw.tps2;
	int throttle_position = 0;
	uint32_t brake_front = adc_raw.front;
	uint32_t brake_rear = adc_raw.rear;
	int brake_pressure = 0;


	int adc_min = SENSOR_MIN * pow(2, ADC_RESOLUTION);
	int adc_max = SENSOR_MAX * pow(2, ADC_RESOLUTION);

	// RANGE CHECK
	if (tps1 < adc_min || tps1 > adc_max) {
		throttleSensorError = true;
	}

	if (tps2 < adc_min || tps2 > adc_max)	{
		throttleSensorError = true;
	}

	if (brake_front < adc_min || brake_front > adc_max) {
		brakeSensorError = true;
	}

	if (brake_rear < adc_min || brake_rear > adc_max) {
		brakeSensorError = true;
	}


	// CONVERT TO POSITION
	tps1 = constrain(tps1, TPS1_MIN, TPS1_MAX);
	tps1 = map(tps1, TPS1_MIN, TPS1_MAX, 0, 100);

	tps2 = constrain(tps2, TPS1_MIN, TPS1_MAX);
	tps2 = map(tps2, TPS2_MIN, TPS2_MAX, 0, 100);

	throttle_position = (tps1 - tps2) / 2;


	// CONVERT TO PRESSURE
	brake_front = constrain(brake_front, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX);
	brake_front = map(brake_front, BRAKE_FRONT_MIN, BRAKE_FRONT_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

	brake_rear = constrain(brake_rear, BRAKE_REAR_MIN, BRAKE_REAR_MAX);
	brake_rear = map(brake_rear, BRAKE_REAR_MIN, BRAKE_REAR_MAX, BRAKE_PRESSURE_MIN, BRAKE_PRESSURE_MAX);

	brake_pressure = (brake_front + brake_rear) / 2;


	// CLEAR APPS BRAKE PLAUSIBILITY IF THROTTLE LESS THAN 5%
	if (throttle_position < 5){
		appsBrakeError = false;
	}


	// CHECK APPS
	if (abs(tps1 - tps2) > 10) {
		appsError = true;
	}


	// CHECK APPS BRK
	if (brake_pressure > BRAKE_PRESSURE_THRESHOLD && throttle_position > 25) {
		appsBrakeError = true;
	}

	appsBrakePlausibilityError = (throttleSensorError || brakeSensorError || appsError || appsBrakeError);

	return appsBrakePlausibilityError;
}

bool checkFault(void)
{
	if (faults.ams || faults.imd || faults.pdoc || faults.bspd) {
		return true;
	}
	else {
		return false;
	}
}

float constrain(float val, float lower, float upper)
{
	float newVal = val;
	if(val < lower) newVal = lower;
	if (val > upper) newVal = upper;

	return newVal;
}

float map(float val, float fromLow, float fromHigh, float toLow, float toHigh)
{
	return (val - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  // PDM Message0
//  if (RxHeader.StdId == CANID_PDMMESSAGE0) {
//	  faults.ams = RxData[0];
//	  faults.pdoc = RxData[1];
//	  faults.imd = RxData[2];
//	  faults.bspd = RxData[3];
//	  pdm_status.precharge = RxData[4];
//	  pdm_status.precharged = RxData[5];
//	  pdm_status.rtd = RxData[6];
//	  pdm_status.drive_enable = RxData[7];
//  }

  if (RxHeader.StdId == CANID_PDMMESSAGE0) {
//	  memcpy(&faults, RxData, sizeof(faults));
	  memcpy(&pdm_status, RxData+4, sizeof(pdm_status));
  }

  if (print_canbus){
	  print_CANBUS_RxData();
  }
}

void TIM1_UP_TIM7_IRQHandler(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7) {
		printf("ADC: ");
		printf("[%d],", adc_raw.tps1);
		printf("[%d],",adc_raw.tps2);
		printf("[%d],", adc_raw.front);
		printf("[%d]\n", adc_raw.rear);
	}
}

void print_CANBUS_RxData() {

	printf("[%d] -  ", RxHeader.StdId);
	printf("RxData: ");
	for (int i = 0; i < RxHeader.DLC; i++){
		printf(" %d,", RxData[i]);
	}
	printf("\n");
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

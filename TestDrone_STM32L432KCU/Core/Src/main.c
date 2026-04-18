/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "MPU6050.h"
#include "LowPassFilter.h"
#include "KalmanRollPitch.h"
#include "PIController.h"
#include "defines.h"
#include "PWMCTRL.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* Peripherals */
MPU6050_t mpu6050;

/* Filters
 /* Low-pass filters */
LPFTwoPole lpf_acc_x, lpf_acc_y, lpf_acc_z;
LPFTwoPole lpf_gyr_x, lpf_gyr_y, lpf_gyr_z;

/* State estimation */
KalmanRollPitch ekf;

/* Controllers */
PIController ctrl_roll;
PIController ctrl_pitch;

/* ESC/Motor control */

ESC_CONF motors;
volatile uint8_t motors_armed = 0;  // Safety flag

/* Timing variables */
uint32_t time_now = 0;
uint32_t time_prev_loop = 0;
uint32_t time_prev_led = 0;

/* --- Sensor Data --- */
float acc_raw[3] = { 0 };
float acc_filtered[3] = { 0 };
float gyr_raw[3] = { 0 };
float gyr_filtered[3] = { 0 };

/* --- Angles & Control --- */
float roll_est = 0.0f;       // Estimated roll (rad)
float pitch_est = 0.0f;      // Estimated pitch (rad)

float roll_setpoint = 0.0f;  // Desired roll (rad)
float pitch_setpoint = 0.0f;

float ctrl_roll_output = 0.0f;
float ctrl_pitch_output = 0.0f;

/* --- Commands (UART/Bluetooth/Zigbee) --- */

/* (Pilot) */
#define CMD_BUFFER_SIZE 10
char cmd_buffer[CMD_BUFFER_SIZE];  // Buffer to store incoming command characters
uint8_t cmd_index = 0;             // Index to keep track of buffer position

volatile int16_t throttle_cmd = 0;
volatile uint8_t rx_pilot_byte;       // Byte received from Phone/PC
volatile float target_pitch = 0.0f;   // Desired Pitch Angle (Radians)
volatile float target_roll = 0.0f;    // Desired Roll Angle (Radians)


/* Zigbee (Agri Sensor) */
#define ZIGBEE_BUFFER_SIZE 128
volatile uint8_t rx_zigbee_byte;      // Byte received from Sensor
char zigbee_buffer[ZIGBEE_BUFFER_SIZE];      // Buffer to store the full message
volatile uint8_t zigbee_idx = 0;      // Buffer index
volatile uint8_t zigbee_msg_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void Modules_Init(void);        // Initialize all sensors and libraries
void Process_Loop_100Hz(void);  // Main Flight Control Algorithm
void Debug_Print(void);         // Send Telemetry via UART
void Send_Web_Log(void);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("\r\n==================================\r\n");
	printf("   STM32 FLIGHT CONTROLLER V1.0   \r\n");
	printf("==================================\r\n");

	// Wait for sensors to power up fully
	HAL_Delay(500);

	/* Initialize Flight Modules */
	Modules_Init();
	/* Initialize Motors */
	ESC_Init(&motors);

	// WARNING: Calibration generates high PWM. Only use when needed.
	//ESC_Calibrate(&motors);

	printf("System Ready. Waiting for ARM command ('A')...\r\n");

	/* Start listening for Bluetooth commands */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_pilot_byte, 1);  // Pilot (BLE)
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &rx_zigbee_byte, 1); // Zigbee

	/* Reset Timing Counters */
	time_prev_loop = HAL_GetTick();
	time_prev_led = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE BEGIN 3 */
		/* Forward the raw data received to Raspberry Pi (UART2)
		 (Expected format for Raspberry Pi: "CH1=%d CH2=%d CH3=%d Temp_DS1621=%.2f C\n")
		 */
		if (zigbee_msg_ready) {

			zigbee_msg_ready = 0;

			char local_copy[ZIGBEE_BUFFER_SIZE];
			strcpy(local_copy, zigbee_buffer);
			zigbee_idx = 0;

			HAL_UART_Transmit(&huart2, (uint8_t*) local_copy,
					strlen(local_copy), 100);
		}
		time_now = HAL_GetTick();
		//--- CONTROL LOOP (100Hz / 10ms) ---  0R 4ms TESTED!!!
		if (time_now - time_prev_loop >= SAMPLE_TIME_CTRL_MS) {
			time_prev_loop = time_now;
			Process_Loop_100Hz();
		}

		// TELEMETRY LOOP (1Hz / 1000ms): SAMPLE_TIME_LED_MS / or (5Hz / 200ms):TELEMETRY_DELAY_MS
		if (time_now - time_prev_led >= TELEMETRY_DELAY_MS) {
			time_prev_led = time_now;
			// Blink LED to indicate "Alive"*******************************VERIFY THIS
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			//Debug_Print();
			Send_Web_Log();
			//printf("Tick=%lu ms\r\n", HAL_GetTick());
		}
		/* USER CODE END 3 */
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/* Forward the raw data received to Raspberry Pi (UART2)
		 (Expected format for Raspberry Pi: "CH1=%d CH2=%d CH3=%d Temp_DS1621=%.2f C\n")

		if (zigbee_msg_ready) {

			zigbee_msg_ready = 0;

			char local_copy[ZIGBEE_BUFFER_SIZE];
			strcpy(local_copy, zigbee_buffer);
			zigbee_idx = 0;

			HAL_UART_Transmit(&huart2, (uint8_t*) local_copy,
					strlen(local_copy), 100);
		}
		time_now = HAL_GetTick();
		//--- CONTROL LOOP (100Hz / 10ms) ---  0R 4ms TESTED!!!
		if (time_now - time_prev_loop >= SAMPLE_TIME_CTRL_MS) {
			time_prev_loop = time_now;
			//Process_Loop_100Hz();
		}

		// TELEMETRY LOOP (1Hz / 1000ms): SAMPLE_TIME_LED_MS / or (5Hz / 200ms):TELEMETRY_DELAY_MS
		if (time_now - time_prev_led >= TELEMETRY_DELAY_MS) {
			time_prev_led = time_now;
			// Blink LED to indicate "Alive"*******************************VERIFY THIS
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
			//Debug_Print();
			Send_Web_Log();
			//printf("Tick=%lu ms\r\n", HAL_GetTick());
		} */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 5 */
void Modules_Init(void) {
	// Initialize MPU6050
	if (MPU6050_Init(&hi2c1) == 0) {
		printf("MPU6050: OK\r\n");
	} else {
		printf("MPU6050: FAILED! Checking connections...\r\n");
		Error_Handler(); // Stop here if sensor fails
	}

	// Initialize Low Pass Filters
	// Accel Cutoff: 10Hz | Gyro Cutoff: 30Hz | Loop Time: 0.01s
	float dt = SAMPLE_TIME_CTRL_MS / 1000.0f;

	LPFTwoPole_Init(&lpf_acc_x, LPF_TYPE_BESSEL, LPF_ACC_CUTOFF_HZ, dt);
	LPFTwoPole_Init(&lpf_acc_y, LPF_TYPE_BESSEL, LPF_ACC_CUTOFF_HZ, dt);
	LPFTwoPole_Init(&lpf_acc_z, LPF_TYPE_BESSEL, LPF_ACC_CUTOFF_HZ, dt);

	LPFTwoPole_Init(&lpf_gyr_x, LPF_TYPE_BESSEL, LPF_GYR_CUTOFF_HZ, dt);
	LPFTwoPole_Init(&lpf_gyr_y, LPF_TYPE_BESSEL, LPF_GYR_CUTOFF_HZ, dt);
	LPFTwoPole_Init(&lpf_gyr_z, LPF_TYPE_BESSEL, LPF_GYR_CUTOFF_HZ, dt);

	// Initialize Kalman Filter
	float Q[2] = { EKF_N_GYR, EKF_N_GYR };
	float R[3] = { EKF_N_ACC, EKF_N_ACC, EKF_N_ACC };
	KalmanRollPitch_Init(&ekf, EKF_P_INIT, Q, R);

	// Initialize PID Controllers
	PI_Init(&ctrl_roll, CTRL_ROLL_P, CTRL_ROLL_I, CTRL_ROLL_LIM_MIN,
	CTRL_ROLL_LIM_MAX);
	PI_Init(&ctrl_pitch, CTRL_PITCH_P, CTRL_PITCH_I, CTRL_PITCH_LIM_MIN,
	CTRL_PITCH_LIM_MAX);

	printf("Modules Initialized.\r\n");
}

/**
 * @brief Main Flight Control Loop (Running at 100Hz)
 * Reads sensors, updates Kalman, runs PID, mixes motors.
 */
void Process_Loop_100Hz(void) {

	// SENSOR READ ---
	MPU6050_Read_Accel(&hi2c1, &mpu6050);
	MPU6050_Read_Gyro(&hi2c1, &mpu6050);

	// PRE-PROCESSING (Filtering & Conversion) ---

	// Accelerometer: Convert raw to m/s^2 (Standard Gravity = 9.81)
	acc_raw[0] = ((mpu6050.Ax - CALIB_ACC_BIAS_X) * CALIB_ACC_SCALE_X) * 9.81f;
	acc_raw[1] = ((mpu6050.Ay - CALIB_ACC_BIAS_Y) * CALIB_ACC_SCALE_Y) * 9.81f;
	acc_raw[2] = ((mpu6050.Az - CALIB_ACC_BIAS_Z) * CALIB_ACC_SCALE_Z) * 9.81f;

	acc_filtered[0] = LPFTwoPole_Update(&lpf_acc_x, acc_raw[0]);
	acc_filtered[1] = LPFTwoPole_Update(&lpf_acc_y, acc_raw[1]);
	acc_filtered[2] = LPFTwoPole_Update(&lpf_acc_z, acc_raw[2]);

	// Gyroscope: Convert Degrees/s to Radians/s
	gyr_raw[0] = mpu6050.Gx * DEG_TO_RAD;
	gyr_raw[1] = mpu6050.Gy * DEG_TO_RAD;
	gyr_raw[2] = mpu6050.Gz * DEG_TO_RAD;

	gyr_filtered[0] = LPFTwoPole_Update(&lpf_gyr_x, gyr_raw[0]);
	gyr_filtered[1] = LPFTwoPole_Update(&lpf_gyr_y, gyr_raw[1]);
	gyr_filtered[2] = LPFTwoPole_Update(&lpf_gyr_z, gyr_raw[2]);

	// ATTITUDE ESTIMATION (Kalman Filter) ---
	float dt = SAMPLE_TIME_CTRL_MS / 1000.0f; // dt = 0.01s

	// Prediction Step (Gyro Integration)
	KalmanRollPitch_Predict(&ekf, gyr_filtered, dt);

	// Update Step (Accelerometer Correction)
	// Va = 0.0f because we are hovering (no airspeed)
	KalmanRollPitch_Update(&ekf, acc_filtered, 0.0f);

	roll_est = ekf.phi;   // Current Roll (Rad)
	pitch_est = ekf.theta; // Current Pitch (Rad)

	// FLIGHT CONTROL (PID) ---

	/*// Setpoints: 0.0 means "Stay Flat"
	//roll_setpoint = 0.0f;
	//pitch_setpoint = 0.0f;*/

	// Apply the setpoints received from Bluetooth
	roll_setpoint  = target_roll;
	pitch_setpoint = target_pitch;

	ctrl_roll_output = PI_Update(&ctrl_roll, roll_setpoint, roll_est, dt);
	ctrl_pitch_output = PI_Update(&ctrl_pitch, pitch_setpoint, pitch_est, dt);

	// MOTOR MIXING & SAFETY ---

	// Convert radians to degrees for safety check
	float roll_deg = roll_est * RAD_TO_DEG;
	float pitch_deg = pitch_est * RAD_TO_DEG;

	// Safety: Emergency Stop if tilted > 45 degrees
	if (!ESC_SafetyCheck(&motors, roll_deg, pitch_deg)) {
		motors_armed = 0;
		throttle_cmd = 0;
	}

	// If Armed, calculate and send PWM
	if (motors_armed) {
		// Mix throttle and PID corrections to 4 motors
		// Yaw correction is set to 0.0f for now (handled by user manually if needed)
		ESC_UpdateCommands(&motors, throttle_cmd, ctrl_roll_output,
				ctrl_pitch_output, 0.0f);
	} else {
		// Ensure motors are OFF if not armed
		ESC_Disarm(&motors);
	}

	// Apply calculated PWM to hardware timers
	ESC_SetSpeed(&motors);
}

/**
 * @brief Prints Telemetry Data to UART
 * Called every 1 second.
 */
void Debug_Print(void)
{
    printf("\r\n===== SENSOR DATA =====\r\n");

    printf("ACC RAW en g :      X=%.2f  Y=%.2f  Z=%.2f g\r\n",
    		mpu6050.Ax, mpu6050.Ay, mpu6050.Az);
    printf("ACC RAW+Calib en m/s² : X=%.2f  Y=%.2f  Z=%.2f m/s²\r\n",
    		acc_raw[0], acc_raw[1], acc_raw[2]);
    printf("ACC FILTERED: X=%.2f  Y=%.2f  Z=%.2f m/s²\r\n",
           acc_filtered[0], acc_filtered[1], acc_filtered[2]);


    printf("GYR RAW en d/s :      X=%.2f  Y=%.2f  Z=%.2f d/s\r\n",
    		mpu6050.Gx, mpu6050.Gy, mpu6050.Gz);
    printf("GYR RAW en rad/s : X=%.2f  Y=%.2f  Z=%.2f rad/s\r\n",
        		gyr_raw[0], gyr_raw[1], gyr_raw[2]);
    printf("GYR FILTERED: X=%.2f  Y=%.2f  Z=%.2f rad/s\r\n",
           gyr_filtered[0], gyr_filtered[1], gyr_filtered[2]);


    printf("\r\n===== KALMAN ESTIMATES =====\r\n");
    printf("Roll:  %.2f°  |  Pitch: %.2f°\r\n",
           roll_est * RAD_TO_DEG,
           pitch_est * RAD_TO_DEG);


    printf("\r\n===== CONTROLLER OUTPUTS =====\r\n");
    printf("Ctrl_Roll:  %.2f  |  Ctrl_Pitch: %.2f\r\n",
           ctrl_roll_output,
           ctrl_pitch_output);

    printf("\r\n===== MOTOR COMMANDS =====\r\n");
    printf("FR: %4d  FL: %4d  RR: %4d  RL: %4d  [%s]\r\n",
               motors.FR, motors.FL, motors.RR, motors.RL,
               motors_armed ? "ARMED" : "DISARMED");
    printf("PWM RANGE: [%d - %d]\r\n", PWM_MIN_US, PWM_MAX_US);

    printf("===============================\r\n\r\n");
}
void Send_Web_Log(void)
{
    char web_msg[256];
    // Format : LOG: [Roll/Pitch] [Motors] [Status]
    sprintf(web_msg, "LOG: ATT: R%.1f P%.1f | MOT: FR:%d FL:%d RR:%d RL:%d | %s\r\n",
            roll_est * RAD_TO_DEG,
            pitch_est * RAD_TO_DEG,
            motors.FR, motors.FL, motors.RR, motors.RL,
            motors_armed ? "ARMED" : "DISARMED");

    // Raspberry Pi (USART2)
    HAL_UART_Transmit(&huart2, (uint8_t*)web_msg, strlen(web_msg), 10);
}

/*void Debug_Print(void) {
	// Print essential data
	// TH: Throttle | R: Roll Angle | P: Pitch Angle | Motors: FR FL RR RL
	printf("TH: %d | Roll: %.1f | Pitch: %.1f | Motors: %d %d %d %d | %s\r\n",
			throttle_cmd, roll_est * RAD_TO_DEG, pitch_est * RAD_TO_DEG,
			motors.FR, motors.FL, motors.RR, motors.RL,
			motors_armed ? "ARMED" : "SAFE");
}*/
void process_command(char *cmd) {
    if(strcmp(cmd, "A") == 0) {
    	motors_armed = 1;
    	ESC_Arm(&motors);
    }           // Arm motors
    else if(strcmp(cmd, "D") == 0) { motors_armed = 0; throttle_cmd = 0; ESC_Disarm(&motors); } // Disarm motors
    else if(strcmp(cmd, "+") == 0) { if (throttle_cmd + 50 <= MOTOR_SAFE_MAX) throttle_cmd += 50; } // Increase throttle
    else if(strcmp(cmd, "-") == 0) { if (throttle_cmd - 50 >= 0) throttle_cmd -= 50; }             // Decrease throttle
    else if(strcmp(cmd, "F") == 0) { target_pitch = -CMD_TILT_ANGLE; target_roll = 0; }             // Forward
    else if(strcmp(cmd, "B") == 0) { target_pitch = CMD_TILT_ANGLE; target_roll = 0; }              // Backward
    else if(strcmp(cmd, "L") == 0) { target_pitch = 0; target_roll = -CMD_TILT_ANGLE; }             // Left
    else if(strcmp(cmd, "R") == 0) { target_pitch = 0; target_roll = CMD_TILT_ANGLE; }              // Right
    else if(strcmp(cmd, "C") == 0) { target_pitch = 0; target_roll = 0; }                            // Center / hover
}
/* USER CODE END 5 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	// --- PILOT CONTROL (Raspberry Pi -> STM32) ---
	if (huart->Instance == USART2) {
	    char c = rx_pilot_byte; // Received byte

	    if (c == '\n') { // End of command detected
	        cmd_buffer[cmd_index] = '\0'; // Null-terminate the string
	        process_command(cmd_buffer);  // Execute command (ARM, DISARM, F, B...)
	        cmd_index = 0;                // Reset for next command
	    }
	    else if (c != '\r') { // Ignore 'Carriage Return'
	        // Store character if there is space
	        if (cmd_index < CMD_BUFFER_SIZE - 1) {
	            cmd_buffer[cmd_index++] = c;
	        }
	    }

	    // Restart UART receive interrupt for next byte
	    HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_pilot_byte, 1);
	}

	// --- ZIGBEE LOGIC (USART1) ---
	else if (huart->Instance == USART1) {
		if (!zigbee_msg_ready) {
			if (zigbee_idx < ZIGBEE_BUFFER_SIZE - 1) {
				if (rx_zigbee_byte != '\r')  // Ignore CR
						{
					zigbee_buffer[zigbee_idx++] = rx_zigbee_byte;
				}

				if (rx_zigbee_byte == '\n') {
					zigbee_buffer[zigbee_idx] = '\0';
					zigbee_msg_ready = 1;
				}
			} else {
				zigbee_idx = 0; // Overflow
			}
		}

		HAL_UART_Receive_IT(&huart1, &rx_zigbee_byte, 1);
	}
}

/**
 * @brief Redirect printf to UART2
 * Allows using printf() for debugging over Bluetooth/USB.
 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
	while (1) {
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

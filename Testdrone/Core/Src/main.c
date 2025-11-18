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

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;// Pour debug via Serial
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* Peripherals */
MPU6050_t mpu6050;

/* Filters
LPFTwoPole lpfAcc[3];
LPFTwoPole lpfGyr[3];
LPFTwoPole lpfMag[3];
LPFTwoPole lpfRC[4];*/
/* Low-pass filters */
LPFTwoPole lpf_acc_x, lpf_acc_y, lpf_acc_z;
LPFTwoPole lpf_gyr_x, lpf_gyr_y, lpf_gyr_z;

/* State estimation */
KalmanRollPitch ekf;

/*
 *float psi;
float psiMag;*/

/* Controllers */
PIController ctrl_roll;
PIController ctrl_pitch;

/* Timing variables */
uint32_t time_now = 0;
uint32_t time_prev_acc = 0;
uint32_t time_prev_gyr = 0;
uint32_t time_prev_ctrl = 0;
uint32_t time_prev_led = 0;

/* State variables */
float acc_raw[3] = {0};      // Accelerometer raw
float acc_filtered[3] = {0}; // Accelerometer filtered
float gyr_raw[3] = {0};      // Gyroscope raw
float gyr_filtered[3] = {0}; // Gyroscope filtered

float roll_est = 0.0f;       // Estimated roll (rad)
float pitch_est = 0.0f;      // Estimated pitch (rad)

float roll_setpoint = 0.0f;  // Desired roll (rad)
float pitch_setpoint = 0.0f; // Desired pitch (rad)

float ctrl_roll_output = 0.0f;
float ctrl_pitch_output = 0.0f;

/// still not develp
/* PWM input
volatile uint32_t pwmRisingEdgeStart[] = {0,0,0,0};
volatile uint32_t pwmOnPeriod[] = {0,0,0,0};
float rcThrottle, rcRoll, rcPitch, rcYaw;
uint8_t running = 0;*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/*  channel 1 interrupt
void EXTI0_IRQHandler(void)
{
    // Vérifier que l'interruption concerne bien Pin 0
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
    {
        // Clear l'interruption
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

        // Allumer toutes les LED (ou tu peux faire toggle)
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

        // Si tu veux toggle à chaque front :
        // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
    }
    // Optionnel : appeler le handler HAL pour compatibilité
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}
*/
/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);

void Sensors_Init(void);
void Filters_Init(void);
void Kalman_Init(void);
void Controllers_Init(void);

void Read_Sensors(void);
void Update_Kalman(void);
void Update_Controllers(void);
void Debug_Print(void);
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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Enable TIM5 for elapsed microseconds count (reading PWM RC input) */
  /*
   *
   *
   *  __HAL_RCC_TIM5_CLK_ENABLE();
    TIM5->PSC = HAL_RCC_GetPCLK1Freq()/1000000 - 1;
    TIM5->CR1 = TIM_CR1_CEN;
    TIM5->ARR = 0xFFFFFFFF;
    TIM5->CNT = 0xFFFFFFFE;  //Dirty fix... PWM values are only correct once timer overruns once. */
  /* Wait for sensor stabilization */
      HAL_Delay(100);

      /* Initialize modules */
      Sensors_Init();
      Filters_Init();
      Kalman_Init();
      Controllers_Init();

      printf("System initialized successfully!\r\n");
      printf("Starting main loop...\r\n\r\n");

      /* Initialize timing */
      time_prev_acc = HAL_GetTick();
      time_prev_gyr = HAL_GetTick();
      time_prev_ctrl = HAL_GetTick();
      time_prev_led = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
      {
          time_now = HAL_GetTick();

          /* Read and filter accelerometer (16ms) */
          if (time_now - time_prev_acc >= SAMPLE_TIME_ACC_MS) {
              time_prev_acc = time_now;

              MPU6050_Read_Accel(&hi2c1, &mpu6050);

              // Apply calibration
              // MPU6050 retourne en 'g', on convertit en m/s²
              acc_raw[0] = ((mpu6050.Ax - CALIB_ACC_BIAS_X) * CALIB_ACC_SCALE_X) * 9.81f;
              acc_raw[1] = ((mpu6050.Ay - CALIB_ACC_BIAS_Y) * CALIB_ACC_SCALE_Y) * 9.81f;
              acc_raw[2] = ((mpu6050.Az - CALIB_ACC_BIAS_Z) * CALIB_ACC_SCALE_Z) * 9.81f;

              // Apply low-pass filter
              acc_filtered[0] = LPFTwoPole_Update(&lpf_acc_x, acc_raw[0]);
              acc_filtered[1] = LPFTwoPole_Update(&lpf_acc_y, acc_raw[1]);
              acc_filtered[2] = LPFTwoPole_Update(&lpf_acc_z, acc_raw[2]);
          }

          /* Read and filter gyroscope (10ms) */
          if (time_now - time_prev_gyr >= SAMPLE_TIME_GYR_MS) {
              time_prev_gyr = time_now;

              MPU6050_Read_Gyro(&hi2c1, &mpu6050);

              // Convert to rad/s
              gyr_raw[0] = mpu6050.Gx * DEG_TO_RAD;
              gyr_raw[1] = mpu6050.Gy * DEG_TO_RAD;
              gyr_raw[2] = mpu6050.Gz * DEG_TO_RAD;

              // Apply low-pass filter
              gyr_filtered[0] = LPFTwoPole_Update(&lpf_gyr_x, gyr_raw[0]);
              gyr_filtered[1] = LPFTwoPole_Update(&lpf_gyr_y, gyr_raw[1]);
              gyr_filtered[2] = LPFTwoPole_Update(&lpf_gyr_z, gyr_raw[2]);

              // Update Kalman filter
              Update_Kalman();
          }

          /* Update controllers (4ms) */
          if (time_now - time_prev_ctrl >= SAMPLE_TIME_CTRL_MS) {
              time_prev_ctrl = time_now;

              Update_Controllers();
          }

          /* Debug print (500ms) */
          if (time_now - time_prev_led >= SAMPLE_TIME_LED_MS) {
              time_prev_led = time_now;

              // Toggle all 4 LEDs on GPIOD (PD12, PD13, PD14, PD15)

              HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
              Debug_Print();
          }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
/* Initialize sensors */
void Sensors_Init(void)
{
    if (MPU6050_Init(&hi2c1) == 0) {
        printf("MPU6050 initialized successfully!\r\n");
    } else {
        printf("ERROR: MPU6050 initialization failed!\r\n");
        Error_Handler();
    }
}

/* Initialize low-pass filters */
void Filters_Init(void)
{
    // Accelerometer filters (10 Hz cutoff, 16ms sample time)
    LPFTwoPole_Init(&lpf_acc_x, LPF_TYPE_BESSEL, LPF_ACC_CUTOFF_HZ, SAMPLE_TIME_ACC_MS / 1000.0f);
    LPFTwoPole_Init(&lpf_acc_y, LPF_TYPE_BESSEL, LPF_ACC_CUTOFF_HZ, SAMPLE_TIME_ACC_MS / 1000.0f);
    LPFTwoPole_Init(&lpf_acc_z, LPF_TYPE_BESSEL, LPF_ACC_CUTOFF_HZ, SAMPLE_TIME_ACC_MS / 1000.0f);

    // Gyroscope filters (32 Hz cutoff, 10ms sample time)
    LPFTwoPole_Init(&lpf_gyr_x, LPF_TYPE_BESSEL, LPF_GYR_CUTOFF_HZ, SAMPLE_TIME_GYR_MS / 1000.0f);
    LPFTwoPole_Init(&lpf_gyr_y, LPF_TYPE_BESSEL, LPF_GYR_CUTOFF_HZ, SAMPLE_TIME_GYR_MS / 1000.0f);
    LPFTwoPole_Init(&lpf_gyr_z, LPF_TYPE_BESSEL, LPF_GYR_CUTOFF_HZ, SAMPLE_TIME_GYR_MS / 1000.0f);

    printf("Low-pass filters initialized!\r\n");
}

/* Initialize Kalman filter */
void Kalman_Init(void)
{
    float Q[2] = {EKF_N_GYR, EKF_N_GYR};  // Process noise
    float R[3] = {EKF_N_ACC, EKF_N_ACC, EKF_N_ACC};  // Measurement noise

    KalmanRollPitch_Init(&ekf, EKF_P_INIT, Q, R);

    printf("Kalman filter initialized!\r\n");
}

/* Initialize PI controllers */
void Controllers_Init(void)
{
    // Roll controller
    PI_Init(&ctrl_roll, CTRL_ROLL_P, CTRL_ROLL_I,
            CTRL_ROLL_LIM_MIN, CTRL_ROLL_LIM_MAX);

    // Pitch controller
    PI_Init(&ctrl_pitch, CTRL_PITCH_P, CTRL_PITCH_I,
            CTRL_PITCH_LIM_MIN, CTRL_PITCH_LIM_MAX);

    printf("PI controllers initialized!\r\n");
}

/* Update Kalman filter */
void Update_Kalman(void)
{
    float dt = SAMPLE_TIME_GYR_MS / 1000.0f;  // 10ms = 0.01s  // ???????????????????????????

    // Predict step (using gyroscope)
    KalmanRollPitch_Predict(&ekf, gyr_filtered, dt);

    // Update step (using accelerometer)
    float Va = 0.0f;  // No airspeed sensor
    uint8_t success = KalmanRollPitch_Update(&ekf, acc_filtered, Va);

    if (success) {
        roll_est = ekf.phi;      // Roll in radians
        pitch_est = ekf.theta;   // Pitch in radians
    }
}

/* Update PI controllers */
void Update_Controllers(void)
{
    float dt = SAMPLE_TIME_CTRL_MS / 1000.0f;  // 4ms = 0.004s // ???????????????????????????

    // TEST: Set fixed setpoints (0 degrees = level flight)// ?????????????????????????????
    roll_setpoint = 0.0f * DEG_TO_RAD;   // 0 degrees
    pitch_setpoint = 0.0f * DEG_TO_RAD;  // 0 degrees

    // Update roll controller
    ctrl_roll_output = PI_Update(&ctrl_roll, roll_setpoint, roll_est, dt);

    // Update pitch controller
    ctrl_pitch_output = PI_Update(&ctrl_pitch, pitch_setpoint, pitch_est, dt);

    // TODO: Apply PWM outputs here when implemented
}

/* Debug print to serial, including raw and filtered values */
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

    printf("===============================\r\n\r\n");
}

/* Printf redirection to UART */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

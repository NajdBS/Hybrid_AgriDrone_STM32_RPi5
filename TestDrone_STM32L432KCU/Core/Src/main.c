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
#include <stdio.h>
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
uint8_t motors_armed = 0;  // Safety flag
uint16_t pwm_fr, pwm_fl, pwm_rr, pwm_rl;


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

/* USER CODE BEGIN PV */
// Déclarations globales
uint8_t rx_data ;
volatile int throttle = 0;

/* USER CODE END PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void Sensors_Init(void);
void Filters_Init(void);
void Kalman_Init(void);
void Controllers_Init(void);

void Read_Sensors(void);
void Update_Kalman(void);
void Update_Controllers(void);
void Debug_Print(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
    	printf("RECEPTION");
        if (rx_data == '+') {
            if (throttle < 600) throttle += 50;
        }
        else if (rx_data == '-') {
            if (throttle > 0) throttle -= 50;
        }
        printf("Throttle %d\r\n", throttle);
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_data, 1);
    }
}
/* USER CODE END 4 */
/* USER CODE END 4 */
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
  /* USER CODE BEGIN 2 */

  /* Wait for sensor stabilization */
       HAL_Delay(100);

       /* Initialize modules */
       Sensors_Init();
       Filters_Init();
       Kalman_Init();
       Controllers_Init();
       // Initialize ESC/Motors
       ESC_Init(&motors);
       ESC_Calibrate(&motors);
       printf("System initialized successfully!\r\n");
       printf("Starting main loop...\r\n\r\n");

       /* Initialize timing */
       time_prev_acc = HAL_GetTick();
       time_prev_gyr = HAL_GetTick();
       time_prev_ctrl = HAL_GetTick();
       time_prev_led = HAL_GetTick();
       /* USER CODE BEGIN 2 */
       // Start listening for ONE character on UART2 using Interrupts
       HAL_UART_Receive_IT(&huart2, &rx_data, 1);

       /* USER CODE END 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100);

	        	    HAL_Delay(4000);

	        	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1100);

	        	   	HAL_Delay(4000);


	        	   	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100);

	        	   	HAL_Delay(4000);

	        	   	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1100);

	        	   	HAL_Delay(4000);*/
	                  time_now = HAL_GetTick();

	                  //Read and filter accelerometer (16ms)
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

	                  // Read and filter gyroscope (10ms)
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

	                  // Update controllers (4ms)
	                  if (time_now - time_prev_ctrl >= SAMPLE_TIME_CTRL_MS) {
	                      time_prev_ctrl = time_now;

	                      Update_Controllers();
	                      motors_armed = 1;
	                      motors.state = ARMED;
	                      Update_Motors();  // Apply PWM to motors
	                  }

	                  // Debug print (500ms)
	                  if (time_now - time_prev_led >= SAMPLE_TIME_LED_MS) {
	                      time_prev_led = time_now;

	                      // Toggle all 4 LEDs on GPIOD (PD12, PD13, PD14, PD15)

	                      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	                      Debug_Print();
	                  }

    /* USER CODE BEGIN 3 */
  }
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
    //

	float dt = SAMPLE_TIME_CTRL_MS / 1000.0f;  // 4ms = 0.004s // ???????????????????????????
	//float dt = SAMPLE_TIME_CTRL_MS;
    // TEST: Set fixed setpoints (0 degrees = level flight)// ?????????????????????????????
    roll_setpoint = 0.0f * DEG_TO_RAD;   // 0 degrees
    pitch_setpoint = 0.0f * DEG_TO_RAD;  // 0 degrees

    // Update roll controller
    ctrl_roll_output = PI_Update(&ctrl_roll, roll_setpoint, roll_est, dt);

    // Update pitch controller
    ctrl_pitch_output = PI_Update(&ctrl_pitch, pitch_setpoint, pitch_est, dt);

    // TODO: Apply PWM outputs here when implemented
}

/* Update Motors based on controller outputs */
void Update_Motors(void)
{
    // SAFETY CHECK: Verify angles are safe
    float roll_deg = roll_est * RAD_TO_DEG;
    float pitch_deg = pitch_est * RAD_TO_DEG;

    /*if (!ESC_SafetyCheck(&motors, roll_deg, pitch_deg)) {
        // Unsafe angles detected - motors already stopped by SafetyCheck
        motors_armed = 0;
        return;
    }*/

    // If not armed, ensure motors are stopped
    if (motors_armed == 0) {
        ESC_Disarm(&motors);
        ESC_SetSpeed(&motors);
        return;
    }

    // Base throttle (for testing, start with 0)
  //  int16_t throttle = 100;  // TODO: Add RC input later

    // Update motor commands based on controller outputs
    // ctrl_roll_output and ctrl_pitch_output are in rad/s from PI controllers
    ESC_UpdateCommands(&motors, throttle, ctrl_roll_output, ctrl_pitch_output, 0.0f);

    // Apply PWM to motors
    ESC_SetSpeed(&motors);
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

    printf("\r\n===== MOTOR COMMANDS =====\r\n");
        printf("FR: %4d  FL: %4d  RR: %4d  RL: %4d  [%s]\r\n",
               motors.FR, motors.FL, motors.RR, motors.RL,
               motors_armed ? "ARMED" : "DISARMED");
        printf("\r\n===== MOTOR PWM OUTPUT =====\r\n");
        printf("PWM_FR: %4d  |  PWM_FL: %4d\r\n", pwm_fr, pwm_fl);
        printf("PWM_RR: %4d  |  PWM_RL: %4d\r\n", pwm_rr, pwm_rl);
        printf("PWM RANGE: [%d - %d]\r\n", PWM_MIN_US, PWM_MAX_US);

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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

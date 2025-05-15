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
#include <string.h>
#include <math.h>
#include "VL53L0X/VL53L0X.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct {
    float Kp, Ki, Kd;
    float prev_error;
    float integral;
} PID;

float pid_update(PID *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LSM9DS1_WHO_AM_I        0x0F
#define LSM9DS1_CTRL_REG1_G     0x10
#define LSM9DS1_CTRL_REG2_G     0x11
#define LSM9DS1_CTRL_REG3_G     0x12
#define LSM9DS1_CTRL_REG4_G     0x13
#define LSM9DS1_CTRL_REG5_XL    0x1F
#define LSM9DS1_CTRL_REG6_XL    0x20
#define LSM9DS1_CTRL_REG8       0x22
#define LSM9DS1_OUT_X_L_G            0x18
#define LSM9DS1_OUT_X_L_XL          0x28


#define CS_AG_GPIO_Port GPIOA
#define CS_AG_Pin GPIO_PIN_4

#define LSM9DS1_CS_LOW() HAL_GPIO_WritePin(CS_AG_GPIO_Port, CS_AG_Pin, GPIO_PIN_RESET)
#define LSM9DS1_CS_HIGH() HAL_GPIO_WritePin(CS_AG_GPIO_Port, CS_AG_Pin, GPIO_PIN_SET)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
// Function
void LSM9DS1_Init();
void LSM9DS1_Read_Gyro(int16_t *x, int16_t *y, int16_t *z);
void LSM9DS1_Read_Accel(int16_t *x, int16_t *y, int16_t *z);
void LSM9DS1_SPI_Read(uint8_t reg, uint8_t *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  /* USER CODE END 2 */
  HAL_Delay(1000);

    LSM9DS1_Init();

    HAL_Delay(500);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

        char uart_msg[128];

        sprintf(uart_msg, "VL53L0X  detected.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

        sprintf(uart_msg, "VL53L0X Test Start\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

        if (!initVL53L0X(1, &hi2c1)) {
            sprintf(uart_msg, "VL53L0X init failed!\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
            while (1); // stop here if failed
        }


  setMeasurementTimingBudget(50000);








    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float ax_g, ay_g, az_g;
    char msg[128];
  //  float ax_g, ay_g, az_g;
    float pitch, roll;

    uint16_t pwm_pulse = 5;

    PID altitude_pid = {5.0f, 0.1f, 0.5f, 0, 0};  // test values



    PID pitch_pid    = {1.2f, 0.05f, 0.3f, 0, 0};
    PID roll_pid     = {1.2f, 0.05f, 0.3f, 0, 0};

    float setpoint_alt = 20.0f;   // target hover height in meters
    float setpoint_pitch = 0.0f;   // level pitch
    float setpoint_roll  = 0.0f;   // level roll
    float dt = 0.02f;



   	    HAL_Delay(20000);

  while (1)
  {




//	  HAL_UART_Transmit(&huart2, (uint8_t*)"Waiting 10 sec before liftoff...\r\n", 34, HAL_MAX_DELAY);
//	     HAL_Delay(5000);  // 10 sec




	  LSM9DS1_Read_Gyro(&gx, &gy, &gz);
	        // Read accelerometer and gyroscope data
	     LSM9DS1_Read_Accel(&ax, &ay, &az);



	      // Print the raw accelerometer data
	      sprintf(msg, "RAW ACC X: %d Y:%d Z:%d\r\n", ax, ay, az);
	      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

	      // Check if raw values are zero
	      if (ax == 0 && ay == 0 && az == 0) {
	          sprintf(msg, "Warning: Raw values are zero!\r\n");
	          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	      }

	      // If raw values are not zero, perform scaling
	      if (ax!=0 || ay != 0 || az != 0) {
	          // Ensure scaling factor is correctly defined
	          const float scaling_factor = 16384.0f;  // ±2g range scaling factor

	          // Scale raw accelerometer values to 'g' by dividing by the scaling factor
	          ax_g = (float)(ax) / scaling_factor;
	          ay_g = (float)(ay) / scaling_factor;
	          az_g = (float)(az) / scaling_factor;

	          // Print the scaled accelerometer values
	          sprintf(msg, "Scaled ACC X: %.4f Y:%.4f Z:%.4f\r\n", ax_g, ay_g, az_g);
	          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	      }

	      HAL_Delay(50);  // Delay for readability

	         // Calculate pitch and roll
	         pitch = atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / 3.14159265359f;
	         roll = atan2f(ay_g, sqrtf(ax_g * ax_g + az_g * az_g)) * 180.0f / 3.14159265359f;

	         // Print pitch and roll values
	         sprintf(msg, "Pitch: %.2f Roll: %.2f\r\n", pitch, roll);
	         HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	         HAL_Delay(50);  // Delay for readability in terminal

	         statInfo_t_VL53L0X stats;



	         uint16_t distance = readRangeSingleMillimeters(&stats);


	  if (timeoutOccurred()) {
		  sprintf(uart_msg, "Sensor timeout!\r\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
	  } else {
		  snprintf(uart_msg, sizeof(uart_msg),
		           "Distance: %.2f cm | Signal: %u | Ambient: %u | Status: %u\r\n",
		           (float)distance / 10.0f,
		           stats.signalCnt, stats.ambientCnt, stats.rangeStatus);
		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
	  }
HAL_Delay(500);


float altitude = (float)distance / 10.0f;  // 1 cm = 10 mm

// PID updates using cm
float base_throttle = 400.0f + pid_update(&altitude_pid, setpoint_alt, altitude, dt);

float pitch_correction  = pid_update(&pitch_pid, setpoint_pitch, pitch, dt);
float roll_correction   = pid_update(&roll_pid, setpoint_roll, roll, dt);

// Motor mixing (adjust if motor rotation direction differs)
int motor_FL = base_throttle + pitch_correction + roll_correction;
int motor_FR = base_throttle + pitch_correction - roll_correction;
int motor_RL = base_throttle - pitch_correction + roll_correction;
int motor_RR = base_throttle - pitch_correction - roll_correction;

// Clamp PWM between 0 and timer period (1599)
motor_FL = fminf(fmaxf(motor_FL, 0), 1599);
motor_FR = fminf(fmaxf(motor_FR, 0), 1599);
motor_RL = fminf(fmaxf(motor_RL, 0), 1599);
motor_RR = fminf(fmaxf(motor_RR, 0), 1599);

// Apply PWM values to motor channels
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_FL);
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor_FR);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor_RL);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor_RR);

// UART debug log in cm
snprintf(msg, sizeof(msg), "Alt: %.1f cm | Pitch: %.2f Roll: %.2f | FL:%d FR:%d RL:%d RR:%d\r\n",
         altitude, pitch, roll, motor_FL, motor_FR, motor_RL, motor_RR);
HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


HAL_Delay(20);

//for (int i = 100; i >= 0; i--)
//  {
//      float ramp = i / 100.0f;
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_FL * ramp);
//      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor_FR * ramp);
//      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor_RL * ramp);
//      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor_RR * ramp);
//      HAL_Delay(40);
//  }
//
//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
//   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);




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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0060112F;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1599;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1599;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void LSM9DS1_SPI_Write(uint8_t reg, uint8_t data) {
	uint8_t tx[2] = { reg & 0x7F, data };
	LSM9DS1_CS_LOW();
	HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
	LSM9DS1_CS_HIGH();
}

void LSM9DS1_SPI_Read(uint8_t reg, uint8_t *data) {
	uint8_t tx = reg | 0x80; // MSB=1 for read
	LSM9DS1_CS_LOW();
	HAL_SPI_Transmit(&hspi1, &tx, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, data, 1, HAL_MAX_DELAY);
	LSM9DS1_CS_HIGH();
}


void LSM9DS1_Read_Accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    uint8_t reg = LSM9DS1_OUT_X_L_XL | 0x80; // OUT_X_L_XL, auto-increment
    LSM9DS1_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, 6, HAL_MAX_DELAY);
    LSM9DS1_CS_HIGH();

    *x = (int16_t)(buffer[1] << 8 | buffer[0]);
    *y = (int16_t)(buffer[3] << 8 | buffer[2]);
    *z = (int16_t)(buffer[5] << 8 | buffer[4]);
}


void LSM9DS1_Read_Gyro(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buffer[6];
    uint8_t reg = LSM9DS1_OUT_X_L_G	 | 0x80; // OUT_X_L_G, auto-increment
    LSM9DS1_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, buffer, 6, HAL_MAX_DELAY);
    LSM9DS1_CS_HIGH();

    *x = (int16_t)(buffer[1] << 8 | buffer[0]);
    *y = (int16_t)(buffer[3] << 8 | buffer[2]);
    *z = (int16_t)(buffer[5] << 8 | buffer[4]);
}



void LSM9DS1_Init() {
    char uart_msg[64];
    uint8_t who_am_i = 0;

    // 1. Soft reset & reboot
    LSM9DS1_SPI_Write(LSM9DS1_CTRL_REG8, 0x05);  // CTRL_REG8 = 0x05

    HAL_Delay(10);  // wait for reset

    // 2. WHO_AM_I check
    LSM9DS1_SPI_Read(0x0F, &who_am_i);
    sprintf(uart_msg, "WHO_AM_I: 0x%02X\r\n", who_am_i);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

    if (who_am_i != 0x68 && who_am_i != 0x6A) {
        sprintf(uart_msg, "LSM9DS1 not found!\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
        return;
    }

    sprintf(uart_msg, "LSM9DS1 detected.\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

    // 3. Enable gyroscope (CTRL_REG1_G)
    LSM9DS1_SPI_Write(LSM9DS1_CTRL_REG1_G, 0xC0);  // 952 Hz, 245 dps, all axes

    // 4. Enable accelerometer axes (CTRL_REG5_XL)
    LSM9DS1_SPI_Write(LSM9DS1_CTRL_REG5_XL, 0x38);  // Enable X, Y, Z

    // 5. Set accelerometer ODR & bandwidth (CTRL_REG6_XL)
    LSM9DS1_SPI_Write(LSM9DS1_CTRL_REG6_XL, 0xC0);  // 952 Hz ODR, 2g, 408 Hz AA BW

    // Enable Block Data Update & little endian format for XL
    LSM9DS1_SPI_Write(LSM9DS1_CTRL_REG8, 0x44);  // CTRL_REG8: BDU=1, IF_ADD_INC=1


    sprintf(uart_msg, "LSM9DS1 Initialization Complete.\r\n\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

    // Optional: CTRL_REG8 again (in case soft reset cleared stuff)
    //LSM9DS1_SPI_Write(0x22, 0x05);  // not needed here anymore
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

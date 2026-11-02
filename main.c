/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- ACS712 settings (CHANGE TO MATCH YOUR MODULE) ----
 * 5A  module = 0.185 V/A
 * 20A module = 0.100 V/A
 * 30A module = 0.066 V/A
 *
 * ACS712 output is ~2.5 V at 0 A (powered from 5 V). The STM32 ADC
 * only accepts 0..3.3 V, so use a voltage divider on the output.
 * ACS712_DIVIDER_RATIO = (R1 + R2) / R2
 *   e.g. R1 = 10k (top), R2 = 20k (bottom)  ->  1.5
 *   no divider                              ->  1.0
 */
#define ACS712_SENSITIVITY_V_PER_A   0.185f
#define ACS712_DIVIDER_RATIO         1.0f
#define ADC_VREF                     3.3f
#define ADC_MAX_COUNT                4095.0f

/* Vibration settings */
#define MPU6050_SAMPLES              50U
#define MPU6050_ACCEL_SCALE          16384.0f   /* +/-2 g range: 16384 LSB/g */
#define VIBRATION_HIGH_THRESHOLD_G   0.50f

/* Status thresholds (adjust to your motor / load) */
#define TEMP_HIGH_THRESHOLD_C        40.0f   /* temperature >= this -> HIGH */
#define CURRENT_HIGH_THRESHOLD_A     1.00f   /* current >= this     -> HIGH (also Speed HIGH) */
#define MOTOR_RUN_CURRENT_A          0.15f   /* current >= this     -> motor RUNNING */

/* Speed controller (potentiometer) read by the STM32
 * Pot middle pin -> PA4 (ADC1_IN4), other pins -> 3.3V and GND.
 * Change these 3 lines if your pot is on another ADC pin.
 */
#define POT_ADC_CHANNEL              ADC_CHANNEL_4
#define POT_GPIO_PORT                GPIOA
#define POT_GPIO_PIN                 GPIO_PIN_4
#define POT_DEADBAND                 100U    /* ADC counts below this -> motors stop */
#define MOTOR_MIN_DUTY               450U    /* starting (normal) speed, 0..999 = 0..100% */
#define MOTOR_PWM_PRESCALER          7U      /* 16 MHz / 8 / 1000 = 2 kHz PWM */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ButtonTask */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
  .name = "ButtonTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* LM35 temperature sensor: PA1 / ADC1_IN1 */
float temperature_celsius = 0.0f;
uint8_t temperature_sensor_ok = 0;

/* ACS712 current sensor: PA0 / ADC1_IN0 */
uint32_t adc_value = 0;
float sensor_voltage = 0.0f;
uint32_t acs_zero_adc = 0;
float acs_current_A = 0.0f;
uint8_t acs_sensor_ok = 0;

/* MPU6050: I2C1
 * PB8 = SCL
 * PB9 = SDA
 */
#define MPU6050_ADDR          (0x68U << 1)
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B

uint16_t mpu6050_addr = MPU6050_ADDR;   /* updated at startup (0x68 or 0x69) */
uint8_t mpu6050_ok = 0;

/* Mutex so only one task uses ADC1 at a time */
osMutexId_t adcMutexHandle = NULL;
const osMutexAttr_t adcMutex_attributes = { .name = "adcMutex" };

uint8_t mpu6050_data[6];

int16_t mpu6050_raw_x = 0;
int16_t mpu6050_raw_y = 0;
int16_t mpu6050_raw_z = 0;

float vibration_x_g = 0.0f;
float vibration_y_g = 0.0f;
float vibration_z_g = 0.0f;
float vibration_magnitude_g = 0.0f;

/* Vibration monitoring */
float vibration_rms_g = 0.0f;

/* Motor-specific indicators:
 * Valid independent measurements require a sensor
 * mounted near each motor.
 */
float motor1_vibration_g = 0.0f;
float motor2_vibration_g = 0.0f;

uint8_t motor1_high_vibration = 0;
uint8_t motor2_high_vibration = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void SensorTask_Entry(void *argument);
void ButtonTask_Entry(void *argument);
void MotorTask_Entry(void *argument);
void MonitorTask_Entry(void *argument);
void CommTask_Entry(void *argument);

/* USER CODE BEGIN PFP */

/* Motor control functions — preserve existing behavior */
void Motor1_SetSpeed(uint16_t speed);
void Motor2_SetSpeed(uint16_t speed);
void Motor_Stop(void);
void Motor_Forward(void);
void Motor_Reverse(void);

/* LM35 temperature sensor */
float LM35_ReadTemperature(void);

/* ACS712 current sensor */
void  ACS712_Calibrate(void);
float ACS712_ReadCurrent(void);

/* MPU6050 sensor functions */
HAL_StatusTypeDef MPU6050_Init(void);
HAL_StatusTypeDef MPU6050_ReadVibration(void);

/* UART output */
void SendSensorDataToESP32(float temperature,
                           float current,
                           float vibration);
void I2C_ScanBus(I2C_HandleTypeDef *hi2c, const char *name);

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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();   /* PuTTY / ST-Link virtual COM port */
  MX_USART6_UART_Init();   /* ESP32 */
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();


   /* USER CODE BEGIN 2 */

  /* 0. Speed-controller pot pin as analog input */
  {
      GPIO_InitTypeDef pot_gpio = {0};
      pot_gpio.Pin  = POT_GPIO_PIN;
      pot_gpio.Mode = GPIO_MODE_ANALOG;
      pot_gpio.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(POT_GPIO_PORT, &pot_gpio);
  }

  /* 1. Start TIM2 base timer */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
      Error_Handler();
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0U);

  /* 1b. Raise motor PWM frequency from ~190 Hz to 2 kHz (HSI = 16 MHz) */
  __HAL_TIM_SET_PRESCALER(&htim1, MOTOR_PWM_PRESCALER);
  htim1.Instance->EGR = TIM_EGR_UG;

  /* 2. Start motor PWM outputs */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
      Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
      Error_Handler();
  }

  /* 3. Keep both motors stopped at startup */
  Motor_Stop();

  /* 4. Test USART2 -> PuTTY */
  const char uart_test[] =
      "\r\n================================\r\n"
      "AKM SQUARE - STM32 STARTED\r\n"
      "USART2 PUTTY TEST OK\r\n"
      "================================\r\n";

  if (HAL_UART_Transmit(
          &huart2,
          (uint8_t *)uart_test,
          (uint16_t)(sizeof(uart_test) - 1U),
          1000U) != HAL_OK)
  {
      Error_Handler();
  }

  /* 5. Check MPU6050 on I2C1 and initialise it */
  const char *mpu_message;

  if (HAL_I2C_IsDeviceReady(
          &hi2c1, (uint16_t)(0x68U << 1), 3U, 100U) == HAL_OK)
  {
      mpu6050_addr = (uint16_t)(0x68U << 1);
      mpu_message = "MPU6050 DETECTED AT 0x68\r\n";
  }
  else if (HAL_I2C_IsDeviceReady(
               &hi2c1, (uint16_t)(0x69U << 1), 3U, 100U) == HAL_OK)
  {
      mpu6050_addr = (uint16_t)(0x69U << 1);
      mpu_message = "MPU6050 DETECTED AT 0x69\r\n";
  }
  else
  {
      mpu_message = "ERROR: MPU6050 NOT DETECTED\r\n";
  }

  if (HAL_UART_Transmit(
          &huart2,
          (uint8_t *)mpu_message,
          (uint16_t)strlen(mpu_message),
          1000U) != HAL_OK)
  {
      Error_Handler();
  }

  if (MPU6050_Init() == HAL_OK)
  {
      mpu6050_ok = 1;
  }

  /* 5b. Diagnostic: list every device found on both I2C buses */
  I2C_ScanBus(&hi2c1, "I2C1");
  I2C_ScanBus(&hi2c2, "I2C2");

  /* 6. Calibrate ACS712 zero-current point (motors are stopped now) */
  ACS712_Calibrate();

  {
      char cal_msg[64];
      int n = snprintf(cal_msg, sizeof(cal_msg),
                       "ACS712 ZERO ADC = %lu\r\n", (unsigned long)acs_zero_adc);
      if (n > 0 && n < (int)sizeof(cal_msg))
      {
          HAL_UART_Transmit(&huart2, (uint8_t *)cal_msg, (uint16_t)n, 1000U);
      }
  }

  /* USER CODE END 2 */




  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  adcMutexHandle = osMutexNew(&adcMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(SensorTask_Entry, NULL, &SensorTask_attributes);

  /* creation of ButtonTask */
  ButtonTaskHandle = osThreadNew(ButtonTask_Entry, NULL, &ButtonTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(MotorTask_Entry, NULL, &MotorTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(MonitorTask_Entry, NULL, &MonitorTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(CommTask_Entry, NULL, &CommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;   /* single conversions (software started) */
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

    /* Additional ADC configuration can go here. */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief USART2 Initialization Function (PuTTY / ST-Link virtual COM port)
  *        PA2 = TX, PA3 = RX
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief USART6 Initialization Function (ESP32)
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|MOTOR1_IN1_Pin|MOTOR1_IN2_Pin
                          |MOTOR2_IN1_Pin|MOTOR2_IN2_Pin|Motor_IN1_Pin|Motor_In2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEMPERATURE_Pin */
  GPIO_InitStruct.Pin = TEMPERATURE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TEMPERATURE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 MOTOR1_IN1_Pin MOTOR1_IN2_Pin
                           MOTOR2_IN1_Pin MOTOR2_IN2_Pin Motor_IN1_Pin Motor_In2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|MOTOR1_IN1_Pin|MOTOR1_IN2_Pin
                          |MOTOR2_IN1_Pin|MOTOR2_IN2_Pin|Motor_IN1_Pin|Motor_In2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* =========================================================
   ADC HELPER
   Reads one ADC1 channel several times and returns the
   average raw (12-bit) value. *ok = 1 on success.
   ========================================================= */
static uint32_t ADC_ReadAveraged_Raw(uint32_t channel, uint32_t samples, uint8_t *ok)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t sum = 0U;
    uint32_t i;

    *ok = 0U;

    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return 0U;
    }

    for (i = 0U; i < samples; i++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            return 0U;
        }

        if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
        {
            HAL_ADC_Stop(&hadc1);
            return 0U;
        }

        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    *ok = 1U;
    return (sum / samples);
}

static uint32_t ADC_ReadAveraged(uint32_t channel, uint32_t samples, uint8_t *ok)
{
    uint8_t locked = 0U;
    uint32_t result;

    if (adcMutexHandle != NULL && osKernelGetState() == osKernelRunning)
    {
        osMutexAcquire(adcMutexHandle, osWaitForever);
        locked = 1U;
    }

    result = ADC_ReadAveraged_Raw(channel, samples, ok);

    if (locked)
    {
        osMutexRelease(adcMutexHandle);
    }

    return result;
}

/* =========================================================
   LM35 TEMPERATURE SENSOR
   LM35 OUT -> PA1 (ADC1_IN1)
   ACS712   -> PA0 (ADC1_IN0)
   ========================================================= */
float LM35_ReadTemperature(void)
{
    uint8_t ok;
    uint32_t raw = ADC_ReadAveraged(ADC_CHANNEL_1, 16U, &ok);

    if (!ok)
    {
        temperature_sensor_ok = 0U;
        return -1000.0f;
    }

    temperature_sensor_ok = 1U;

    /* Convert ADC result into voltage */
    float voltage = ((float)raw * ADC_VREF) / ADC_MAX_COUNT;

    /* LM35 output: typically 10 mV per degree Celsius */
    return voltage * 100.0f;
}


/* =========================================================
   ACS712 CURRENT SENSOR (PA0 / ADC1_IN0)
   Zero-current point is measured at startup with motors off.
   ========================================================= */
void ACS712_Calibrate(void)
{
    uint8_t ok;
    uint32_t raw = ADC_ReadAveraged(ADC_CHANNEL_0, 200U, &ok);

    if (ok)
    {
        acs_zero_adc = raw;
        acs_sensor_ok = 1U;
    }
    else
    {
        acs_sensor_ok = 0U;
    }
}

float ACS712_ReadCurrent(void)
{
    uint8_t ok;
    uint32_t raw = ADC_ReadAveraged(ADC_CHANNEL_0, 32U, &ok);

    if (!ok)
    {
        acs_sensor_ok = 0U;
        return 0.0f;
    }

    acs_sensor_ok = 1U;
    adc_value = raw;

    /* Voltage at the ADC pin */
    sensor_voltage = ((float)raw * ADC_VREF) / ADC_MAX_COUNT;

    /* Voltage change at the ACS712 output (undo the divider) */
    float delta_v = (((float)raw - (float)acs_zero_adc) * ADC_VREF / ADC_MAX_COUNT)
                    * ACS712_DIVIDER_RATIO;

    float current = delta_v / ACS712_SENSITIVITY_V_PER_A;

    /* Ignore tiny noise around zero */
    if (fabsf(current) < 0.05f)
    {
        current = 0.0f;
    }

    return current;
}


/* =========================================================
   MOTOR 1 PWM SPEED CONTROL
   TIM1 CHANNEL 1
   ========================================================= */

void Motor1_SetSpeed(uint16_t speed)
{
    if (speed > 999)
        speed = 999;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}


/* =========================================================
   MOTOR 2 PWM SPEED CONTROL
   TIM1 CHANNEL 2
   ========================================================= */

void Motor2_SetSpeed(uint16_t speed)
{
    if (speed > 999)
        speed = 999;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}


/* =========================================================
   STOP BOTH MOTORS
   ========================================================= */

void Motor_Stop(void)
{
    Motor1_SetSpeed(0);
    Motor2_SetSpeed(0);
}


/* =========================================================
   BOTH MOTORS FORWARD
   Motor 1: PB12 / PB13
   Motor 2: PB14 / PB15
   ========================================================= */

void Motor_Forward(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}


/* =========================================================
   BOTH MOTORS REVERSE
   ========================================================= */

void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}


/* =========================================================
   MPU6050 INITIALIZATION
   I2C1: PB8 SCL, PB9 SDA
   Wakes the sensor and selects the +/-2 g range.
   ========================================================= */
HAL_StatusTypeDef MPU6050_Init(void)
{
    uint8_t wake = 0x00;       /* clear sleep bit */
    uint8_t range = 0x00;      /* +/-2 g */
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Write(&hi2c1, mpu6050_addr, MPU6050_PWR_MGMT_1,
                           I2C_MEMADD_SIZE_8BIT, &wake, 1U, 100U);
    if (st != HAL_OK)
    {
        return st;
    }

    st = HAL_I2C_Mem_Write(&hi2c1, mpu6050_addr, MPU6050_ACCEL_CONFIG,
                           I2C_MEMADD_SIZE_8BIT, &range, 1U, 100U);
    return st;
}


/* =========================================================
   MPU6050 VIBRATION READING
   Takes MPU6050_SAMPLES accelerometer samples and computes the
   RMS of the acceleration magnitude around its mean. Because
   the mean is removed, gravity (1 g) does not affect the result.
   Result -> vibration_rms_g
   Must be called from a task (uses osDelay).
   ========================================================= */
HAL_StatusTypeDef MPU6050_ReadVibration(void)
{
    float sum = 0.0f;
    float sum_sq = 0.0f;
    uint32_t i;

    for (i = 0U; i < MPU6050_SAMPLES; i++)
    {
        HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, mpu6050_addr,
                                                MPU6050_ACCEL_XOUT_H,
                                                I2C_MEMADD_SIZE_8BIT,
                                                mpu6050_data, 6U, 100U);
        if (st != HAL_OK)
        {
            return st;
        }

        mpu6050_raw_x = (int16_t)((mpu6050_data[0] << 8) | mpu6050_data[1]);
        mpu6050_raw_y = (int16_t)((mpu6050_data[2] << 8) | mpu6050_data[3]);
        mpu6050_raw_z = (int16_t)((mpu6050_data[4] << 8) | mpu6050_data[5]);

        vibration_x_g = (float)mpu6050_raw_x / MPU6050_ACCEL_SCALE;
        vibration_y_g = (float)mpu6050_raw_y / MPU6050_ACCEL_SCALE;
        vibration_z_g = (float)mpu6050_raw_z / MPU6050_ACCEL_SCALE;

        vibration_magnitude_g = sqrtf(vibration_x_g * vibration_x_g +
                                      vibration_y_g * vibration_y_g +
                                      vibration_z_g * vibration_z_g);

        sum    += vibration_magnitude_g;
        sum_sq += vibration_magnitude_g * vibration_magnitude_g;

        osDelay(2);
    }

    float mean = sum / (float)MPU6050_SAMPLES;
    float var  = (sum_sq / (float)MPU6050_SAMPLES) - (mean * mean);

    if (var < 0.0f)
    {
        var = 0.0f;
    }

    vibration_rms_g = sqrtf(var);

    return HAL_OK;
}


/* =========================================================
   I2C BUS SCAN (diagnostic) - prints every device found
   ========================================================= */
void I2C_ScanBus(I2C_HandleTypeDef *hi2c, const char *name)
{
    char line[96];
    int n = snprintf(line, sizeof(line), "%s devices:", name);
    uint8_t found = 0U;
    uint16_t addr;

    for (addr = 1U; addr < 127U; addr++)
    {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1U, 5U) == HAL_OK)
        {
            if (n > 0 && n < (int)sizeof(line) - 8)
            {
                n += snprintf(line + n, sizeof(line) - (size_t)n, " 0x%02X", (unsigned)addr);
            }
            found = 1U;
        }
    }

    if (!found && n > 0 && n < (int)sizeof(line) - 8)
    {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " NONE");
    }

    if (n > 0 && n < (int)sizeof(line) - 3)
    {
        line[n++] = '\r';
        line[n++] = '\n';
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 1000U);
    }
}


/* =========================================================
   FLOAT -> TEXT without needing printf float support
   e.g. FormatFixed(buf, size, 29.4f, 2) -> "29.40"
   ========================================================= */
static void FormatFixed(char *buf, size_t size, float value, uint8_t decimals)
{
    uint32_t scale = 1U;
    uint8_t i;
    uint8_t neg = (value < 0.0f) ? 1U : 0U;

    for (i = 0U; i < decimals; i++)
    {
        scale *= 10U;
    }

    if (neg)
    {
        value = -value;
    }

    uint32_t scaled = (uint32_t)(value * (float)scale + 0.5f);

    if (scaled == 0U)
    {
        neg = 0U;
    }

    snprintf(buf, size, "%s%lu.%0*lu",
             neg ? "-" : "",
             (unsigned long)(scaled / scale),
             (int)decimals,
             (unsigned long)(scaled % scale));
}


/* =========================================================
   SEND SENSOR REPORT
   -> PuTTY (USART2) and ESP32 (USART6)
   ========================================================= */
void SendSensorDataToESP32(float temperature,
                           float current,
                           float vibration)
{
    static char tx_buffer[384];
    char num[20];
    char temp_line[40];
    char cur_line[40];
    char vib_line[48];

    /* Temperature */
    if (temperature_sensor_ok)
    {
        FormatFixed(num, sizeof(num), temperature, 2);
        snprintf(temp_line, sizeof(temp_line), "%s C (%s)", num,
                 (temperature >= TEMP_HIGH_THRESHOLD_C) ? "HIGH" : "LOW");
    }
    else
    {
        snprintf(temp_line, sizeof(temp_line), "SENSOR ERROR");
    }

    /* Current */
    if (acs_sensor_ok)
    {
        FormatFixed(num, sizeof(num), current, 3);
        snprintf(cur_line, sizeof(cur_line), "%s A (%s)", num,
                 (current >= CURRENT_HIGH_THRESHOLD_A) ? "HIGH" : "LOW");
    }
    else
    {
        snprintf(cur_line, sizeof(cur_line), "SENSOR ERROR");
    }

    /* Vibration */
    if (mpu6050_ok)
    {
        FormatFixed(num, sizeof(num), vibration, 3);
        snprintf(vib_line, sizeof(vib_line), "%s g (%s)", num,
                 (vibration >= VIBRATION_HIGH_THRESHOLD_G) ? "HIGH" : "LOW");
    }
    else
    {
        /* MPU6050 not available: estimate level from motor current */
        snprintf(vib_line, sizeof(vib_line), "%s (est. from current)",
                 (current >= CURRENT_HIGH_THRESHOLD_A) ? "HIGH" : "LOW");
    }

    /* Motor state and speed level (speed follows the motor current) */
    uint32_t pwm = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1)
                 + __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);

    uint8_t running = (current >= MOTOR_RUN_CURRENT_A) || (pwm > 0U);

    const char *motor_str = running ? "RUNNING" : "STOPPED";
    const char *speed_str = !running ? "OFF" :
                            ((current >= CURRENT_HIGH_THRESHOLD_A) ? "HIGH" : "LOW");

    int len = snprintf(
        tx_buffer,
        sizeof(tx_buffer),
        "\r\n"
        "====================================\r\n"
        "       AKM SQUARE SENSOR MONITOR\r\n"
        "====================================\r\n"
        "Temperature : %s\r\n"
        "Current     : %s\r\n"
        "Vibration   : %s\r\n"
        "Motor       : %s\r\n"
        "Speed       : %s\r\n"
        "====================================\r\n",
        temp_line,
        cur_line,
        vib_line,
        motor_str,
        speed_str
    );

    if (len > 0 && len < (int)sizeof(tx_buffer))
    {
        /* Send to PC / PuTTY through USART2 */
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_buffer, (uint16_t)len, 1000U);

        /* Send to ESP32 through USART6 */
        HAL_UART_Transmit(&huart6, (uint8_t *)tx_buffer, (uint16_t)len, 1000U);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SensorTask_Entry */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorTask_Entry */
void SensorTask_Entry(void *argument)
{
    /* USER CODE BEGIN SensorTask_Entry */

    for (;;)
    {
        /* 1. Temperature (LM35) */
        temperature_celsius = LM35_ReadTemperature();

        /* 2. Current (ACS712) */
        acs_current_A = ACS712_ReadCurrent();

        /* 3. Vibration (MPU6050) - retry init if it was not found earlier */
        if (!mpu6050_ok)
        {
            if (MPU6050_Init() == HAL_OK)
            {
                mpu6050_ok = 1U;
            }
        }

        if (mpu6050_ok)
        {
            if (MPU6050_ReadVibration() != HAL_OK)
            {
                mpu6050_ok = 0U;
            }
        }

        /* 4. Print to PuTTY (USART2) and ESP32 (USART6) */
        SendSensorDataToESP32(temperature_celsius,
                              acs_current_A,
                              vibration_rms_g);

        osDelay(1000);
    }

    /* USER CODE END SensorTask_Entry */
}

/* USER CODE BEGIN Header_ButtonTask_Entry */
/**
* @brief Function implementing the ButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ButtonTask_Entry */
void ButtonTask_Entry(void *argument)
{
    /* USER CODE BEGIN ButtonTask_Entry */

    for (;;)
    {
        /* Button handling can be added here. */
        osDelay(50);
    }

    /* USER CODE END ButtonTask_Entry */
}


/* USER CODE BEGIN Header_MotorTask_Entry */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorTask_Entry */
void MotorTask_Entry(void *argument)
{
    /* USER CODE BEGIN MotorTask_Entry */

    for (;;)
    {
        /* Speed controller (pot) -> PWM duty for both motors */
        uint8_t ok;
        uint32_t raw = ADC_ReadAveraged(POT_ADC_CHANNEL, 8U, &ok);

        if (ok)
        {
            if (raw < POT_DEADBAND)
            {
                Motor_Stop();
            }
            else
            {
                /* Starts at normal speed (MOTOR_MIN_DUTY), reaches 100% at full pot */
                uint32_t span = 4095U - POT_DEADBAND;
                uint16_t duty = (uint16_t)(MOTOR_MIN_DUTY +
                                ((raw - POT_DEADBAND) * (999U - MOTOR_MIN_DUTY)) / span);

                Motor_Forward();
                Motor1_SetSpeed(duty);
                Motor2_SetSpeed(duty);
            }
        }

        osDelay(50);
    }

    /* USER CODE END MotorTask_Entry */
}

/* USER CODE BEGIN Header_MonitorTask_Entry */
/**
* @brief Function implementing the MonitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MonitorTask_Entry */
void MonitorTask_Entry(void *argument)
{
  /* USER CODE BEGIN MonitorTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MonitorTask_Entry */
}

/* USER CODE BEGIN Header_CommTask_Entry */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommTask_Entry */
void CommTask_Entry(void *argument)
{
  /* USER CODE BEGIN CommTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CommTask_Entry */
}

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
  if (htim->Instance == TIM6)
  {
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

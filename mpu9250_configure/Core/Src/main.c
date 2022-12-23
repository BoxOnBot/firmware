/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu_9250.h"
#include "printf.h"
#include "Fusion/Fusion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_RATE (100)
#define MSG_MAX_LEN 120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  dataHandleIMU himu1;

  char msg[MSG_MAX_LEN] = {'\0'};
  uint16_t msgLen = 0;

  uint32_t prevPoll = 0;
  uint32_t currPoll = 0;
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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU_9250_Init();

  // Define calibration
  // sensitivity and alignment taken into account in mpu_9250.c
  const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
  const FusionVector gyroscopeOffset = {26.17f, -9.3f, -3.0f};
  const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
  const FusionVector accelerometerOffset = {-0.005f, 0.025f, -0.07f};
  const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

  // Initialise algorithms
  FusionOffset offset;
  FusionAhrs ahrs;

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
          .gain = 1.0f,
          .accelerationRejection = 10.0f,
          .magneticRejection = 20.0f,
          .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    currPoll = HAL_GetTick();
    
    if ((currPoll - prevPoll) >= (1/SAMPLE_RATE)*1000){
      IMU_measure(&himu1);

      FusionVector gyroscope = {himu1.gx, himu1.gy, himu1.gz}; // gyroscope data in degrees/s
      FusionVector accelerometer = {himu1.ax, himu1.ay, himu1.az}; // accelerometer data in g
      FusionVector magnetometer = {himu1.mx, himu1.my, himu1.mz}; // magnetometer data in arbitrary units

      // Apply calibration
      gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
      accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
      magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

      // swap imu axes to axes of system
      gyroscope = FusionAxesSwap(gyroscope, FusionAxesAlignmentPYNXPZ);
      accelerometer = FusionAxesSwap(accelerometer, FusionAxesAlignmentPYNXPZ);
      magnetometer = FusionAxesSwap(magnetometer, FusionAxesAlignmentPYNXPZ);

      // Update gyroscope offset correction algorithm
      gyroscope = FusionOffsetUpdate(&offset, gyroscope);

      // Calculate delta time (in seconds) to account for gyroscope sample clock error
      const float deltaTime = (float) (currPoll - prevPoll) / (float) 1000;

      // Update gyroscope AHRS algorithm
      // FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

      FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

      // algorithm outputs
      const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
      // const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

      // print algo outputs
      // msgLen = sprintf_(msg, "[[%0.2f, %0.2f, %0.2f],[%0.2f, %0.2f, %0.2f],[%0.2f, %0.2f, %0.2f]]\n", himu1.ax, himu1.ay, himu1.az, himu1.gx, himu1.gy, himu1.gz, himu1.mx, himu1.my, himu1.mz);

      msgLen = sprintf_(msg,"Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

      HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)msgLen, 0xFFFF);

      prevPoll = currPoll;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */

// legacy dump

// sprintf_(msg, ">%0.2f, %0.2f, %0.2f;%0.2f, %0.2f, %0.2f;%0.2f, %02f, %0.2f<\n", himu1.ax, himu1.ay, himu1.az, himu1.gx, himu1.gy, himu1.gz, himu1.mx, himu1.my, himu1.mz);

// msgLen = sprintf_(msg, "[[%0.2f, %0.2f, %0.2f],[%0.2f, %0.2f, %0.2f]]\r\n", himu1.ax, himu1.ay, himu1.az, himu1.gx, himu1.gy, himu1.gz);

// msgLen = sprintf_(msg, "[[%0.2f, %0.2f, %0.2f]]\r\n", himu1.ax, himu1.ay, himu1.az);

// HAL_UART_Transmit_IT(&huart3, (uint8_t *)msg, (uint16_t)100);

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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>

#include <stdbool.h>
#include <usart.h>

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
  // Define calibration
  // sensitivity and alignment taken into account in mpu_9250.c
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {103.0f, -35.0f, -10.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.05f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;
FusionQuaternion quarternion;

FusionVector gyroscope; // gyroscope data in degrees/s
FusionVector accelerometer; // accelerometer data in g
FusionVector magnetometer; // magnetometer data in arbitrary units


// Set AHRS algorithm settings
const FusionAhrsSettings settings = {
        .gain = 1.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
};

dataHandleIMU himu1;

uint32_t prevPoll = 0;
uint32_t currPoll = 0;

double cov[9] = {0.05f, 0, 0, 0, 0.05f, 0, 0, 0, 0.05f};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 800 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  FusionAhrsSetSettings(&ahrs, &settings);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  rmw_uros_set_custom_transport(
  true,
  (void *) &huart1,
  cubemx_transport_open,
  cubemx_transport_close,
  cubemx_transport_write,
  cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  rcutils_set_default_allocator(&freeRTOS_allocator);


  // micro-ROS app

  rcl_publisher_t publisher;

  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;

  volatile rcl_ret_t ret;
  volatile bool err = true;

  std_msgs__msg__Int32 msg;

  sensor_msgs__msg__Imu msg_imu;
  rosidl_runtime_c__String frame_name;
  frame_name.data = "bob_imu";

  allocator = rcl_get_default_allocator();

  //create init_options
  ret = rclc_support_init(&support, 0, NULL, &allocator);
  if (ret != RCL_RET_OK)
  {
    err = false; 
  }
  // create node
  const char* node_name = "bifrost_node";
  const char * namespace = "";
  ret = rclc_node_init_default(&node, node_name, namespace, &support);
  if (ret != RCL_RET_OK)
  {
    err = false; 
  }

  // create publisher 
  const char* topic_name = "IMU_data";
  ret = rclc_publisher_init_default(
  &publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  topic_name);
  if (ret != RCL_RET_OK)
  {
    err = false; 
  }

  msg.data = 0;

  msg_imu.header.frame_id = frame_name;
  msg_imu.orientation.w = 1;
  msg_imu.orientation.x = 0;
  msg_imu.orientation.y = 0;
  msg_imu.orientation.z = 0;

  msg_imu.angular_velocity.x = 0;
  msg_imu.angular_velocity.y = 0;
  msg_imu.angular_velocity.z = 0;

  msg_imu.linear_acceleration.x = 0;
  msg_imu.linear_acceleration.y = 0;
  msg_imu.linear_acceleration.z = 0;

  for (int i = 0; i < 9; i++){
    msg_imu.orientation_covariance[i] = cov[i];
    msg_imu.angular_velocity_covariance[i] = cov[i];
    msg_imu.linear_acceleration_covariance[i] = cov[i];
  }

  for(;;)
  {
    currPoll = HAL_GetTick();
    
    if ((currPoll - prevPoll) >= (1/SAMPLE_RATE)*1000){
      IMU_measure(&himu1);

      // set values from imu
      gyroscope.axis.x = himu1.gx;
      gyroscope.axis.y = himu1.gy;
      gyroscope.axis.z = himu1.gz; 

      accelerometer.axis.x = himu1.ax / 3.7f;
      accelerometer.axis.y = himu1.ay / 3.7f;
      accelerometer.axis.z = himu1.az / 3.7f; 

      magnetometer.axis.x = himu1.mx;
      magnetometer.axis.y = himu1.my;
      magnetometer.axis.z = himu1.mz; 

      // Apply calibration
      gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
      accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
      magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

      // swap imu axes to axes of system
      // gyroscope = FusionAxesSwap(gyroscope, FusionAxesAlignmentPYNXPZ);
      // accelerometer = FusionAxesSwap(accelerometer, FusionAxesAlignmentPYNXPZ);
      // magnetometer = FusionAxesSwap(magnetometer, FusionAxesAlignmentPYNXPZ);

      // Update gyroscope offset correction algorithm
      gyroscope = FusionOffsetUpdate(&offset, gyroscope);

      // Calculate delta time (in seconds) to account for gyroscope sample clock error
      volatile float deltaTime = (float) (currPoll - prevPoll) / (float) 1000;

      // Update AHRS algorithm
      FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

      // algorithm outputs
      quarternion = FusionAhrsGetQuaternion(&ahrs);
    }

    msg.data++;
    msg_imu.orientation.w = quarternion.element.w;
    msg_imu.orientation.x = quarternion.element.x;
    msg_imu.orientation.y = quarternion.element.y;
    msg_imu.orientation.z = quarternion.element.z;

    msg_imu.angular_velocity.x = gyroscope.axis.x;
    msg_imu.angular_velocity.y = gyroscope.axis.y;
    msg_imu.angular_velocity.z = gyroscope.axis.z;

    msg_imu.linear_acceleration.x = accelerometer.axis.x;
    msg_imu.linear_acceleration.y = accelerometer.axis.y;
    msg_imu.linear_acceleration.z = accelerometer.axis.z;

    // rcl_ret_t ret = rcl_publish(&publisher, &msg_imu, NULL);
    // if (ret != RCL_RET_OK)
    // {
    //   // printf("Error publishing (line %d)\n", __LINE__); 
    // }

    ret = rcl_publish(&publisher, &msg_imu, NULL);
    if (ret != RCL_RET_OK)
    {
      err = false; 
    }


    osDelay(10);
    prevPoll = currPoll;
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


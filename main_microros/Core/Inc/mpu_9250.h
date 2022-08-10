/*
 * mpu_9250.h
 *
 *  Created on: Jun 29, 2022
 *      Author: jasshank
 */

#ifndef INC_MPU_9250_H_
#define INC_MPU_9250_H_

#include "main.h"
#include "i2c.h"

#define AK8963_SLAVE_ADDRESS (uint8_t)0x18 // magnetometer
#define MPU9250_SLAVE_ADDRESS (uint8_t)0xD0 // AD0 = 0, 0b1101000X

static float sensAccel = 0.0f;
static float sensGyro = 0.0f;
static float sensMag = 0.146f;
static float biasAccel[3];
static float biasGyro[3];
static float biasMag[3];

static uint8_t mag_sens = 0;

static void MPU_9250_Init(void);
static void Accel_readXYZ(float* data);
static void Gyro_readXYZ(float *data);
static void Magneto_readXYZ(float *data);

#endif /* INC_MPU_9250_H_ */

/*
 * mpu_9250.h
 *
 *  Created on: Jun 29, 2022
 *      Author: jasshank
 */

#ifndef INC_MPU_9250_H_
#define INC_MPU_9250_H_

#include <stdint.h>

#define AK8963_SLAVE_ADDRESS (uint8_t)0x18 // magnetometer
#define MPU9250_SLAVE_ADDRESS (uint8_t)0xD0 // AD0 = 0, 0b1101000X

static float sensAccel = 0.0f;
static float sensGyro = 0.0f;
static float sensMag = 0.146f;

// static float biasAccel[3];
// static float biasGyro[3];
// static float biasMag[3];
static float sensAdjMag[3];

static uint8_t mag_sens = 0; 

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz; 
} dataHandleIMU;

void MPU_9250_Init(void);
void Accel_readXYZ(dataHandleIMU* data);
void Gyro_readXYZ(dataHandleIMU *data);
void Magneto_readXYZ(dataHandleIMU *data);
void IMU_measure(dataHandleIMU* data);

#endif /* INC_MPU_9250_H_ */

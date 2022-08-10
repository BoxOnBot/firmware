#include "mpu_9250.h"

static void MPU_9250_Init(void){
	uint8_t tmp = 0;
	// uint8_t buffer[3];

	// user_ctrl, signal reset
	tmp = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x6A, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// WHO_AM_I
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x75, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);
//	if (tmp == 0x71){
//		static char msgTest[] = "MPU9250 initialising\r\n";
//		USART2_print(msgTest);
//	}

	// pwr_mgmt_1, clock source config
	tmp = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x6B, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// config
	tmp = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x1A, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// gyro_config, 500 dps, filter
	tmp = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x1B, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// accel_config, 2g
	tmp = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x1C, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// accel_config_2, filter
	tmp = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x1D, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// int_pin_cfg, bypass enabled
	tmp = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x37, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// pwr_mgmt_2, enable accelerometer and gyroscope
	tmp = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x6C, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// magneto cntl2, soft reset
	tmp = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_SLAVE_ADDRESS, 0x0B, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// magneto cntl1, continuous mode 1, 16 bit output
	tmp = 0x12;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_SLAVE_ADDRESS, 0x0A, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// magneto sensitivity adjustment
	// if (mag_sens == 1){
	// 	HAL_I2C_Mem_Read(&hi2c1, AK8963_SLAVE_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, buffer, 0x3, 1000);

	// 	for (int i = 0; i < 3; i++){
	// 		sensAdjMag[i] = (((buffer[i] - 128) * 0.5)/128) + 1;
	// 	}
	// }

	// checking accel scale
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x1C, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// select accel sensitivity for respective scale
	switch(tmp & 0x18){
	case 0x00:
		sensAccel = 16384.0f;
		break;
	case 0x08:
		sensAccel = 8192.0f;
		break;
	case 0x10:
		sensAccel = 4096.0f;
		break;
	case 0x18:
		sensAccel = 2048.0f;
		break;
	}

	// checking gyro scale
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x1B, I2C_MEMADD_SIZE_8BIT, &tmp, 0x1, 1000);
	HAL_Delay(5);

	// select gyro sens
	switch(tmp & 0x18){
	case 0x00:
		sensGyro = 131.0f;
		break;
	case 0x08:
		sensGyro = 65.5f;
		break;
	case 0x10:
		sensGyro = 32.8f;
		break;
	case 0x18:
		sensGyro = 16.4f;
		break;
	}
}

static void Accel_readXYZ(float* data){
	uint8_t buffer[6];
	int16_t rawData[3];

	// read accelerometer values
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, 0x6, 1000);
	HAL_Delay(5);

	// merging data
	for (int i = 0; i < 3; i++){
		rawData[i] = ((int16_t)(buffer[2*i] << 8) | buffer[2*i+1]);
	}

	// scaling to appropriate unit
	for (int i = 0; i < 3; i++){
		data[i] = (float)(rawData[i] / sensAccel);
	}
}

static void Gyro_readXYZ(float *data){
	uint8_t buffer[6];
	int16_t rawData[3];

	// read gyro values
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_SLAVE_ADDRESS, 0x43, I2C_MEMADD_SIZE_8BIT, buffer, 0x6, 1000);
	HAL_Delay(5);

	// merging data
	for (int i = 0; i < 3; i++){
		rawData[i] = ((int16_t)(buffer[2*i] << 8) | buffer[2*i+1]);
	}

	// scaling to appropriate unit
	for (int i = 0; i < 3; i++){
		data[i] = (float)(rawData[i] / sensGyro);
	}
}

// Y, X, -Z
static void Magneto_readXYZ(float *data){
	uint8_t buffer[6];
	int16_t rawData[3];
	float tmp;

	// read gyro values
	HAL_I2C_Mem_Read(&hi2c1, AK8963_SLAVE_ADDRESS, 0x03, I2C_MEMADD_SIZE_8BIT, buffer, 0x6, 1000);
	HAL_Delay(5);

	// merging data
	for (int i = 0; i < 3; i++){
		rawData[i] = ((int16_t)(buffer[2*i+1] << 8) | buffer[2*i]);
	}

	// scaling to appropriate unit
	for (int i = 0; i < 3; i++){
		data[i] = (float)(rawData[i] * sensMag);
	}

	// correcting axis
	tmp = data[0];
	data[0] = data[1];
	data[1] = tmp;
	data[2] = -data[2];
}

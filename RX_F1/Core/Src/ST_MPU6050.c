/*
Library:                MPU6050 - Accelerometer and gyroscope -  for ST Microntroller
Written by:             Sang Truong Tan
Date Written:           03/26/2021
Last modified:      04/06/2021
Description:
References:
            1) STM Devices

            2) MPU6050 datasheet
                  https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

*Copyright (C) 2021 - TTSang
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/


/********************* Header files *********************/
#include        "ST_MPU6050.h"


/********************* Global Variable *********************/
float Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_2G;

float Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_250DPS;

I2C_HandleTypeDef mpuhi2c;
/******************** Function Definitions *********************/

//1. Read Register
void MPU6050_Read (uint8_t* ui8pDataR, uint8_t ui8Add, uint8_t ui8size)
{
     HAL_I2C_Mem_Read(&mpuhi2c, MPU6050_Adress, ui8Add, 1, ui8pDataR, ui8size, 10);
}

//2. Write Register
void MPU6050_Write (uint8_t* ui8pDataW, uint8_t ui8Add, uint8_t ui8size)
{
    HAL_I2C_Mem_Write(&mpuhi2c, MPU6050_Adress, ui8Add, 1, ui8pDataW, ui8size, 10);
}
//3. MPU6050 Initialize

MPU6050_State MPU6050_Init (MPU6050_InitTypedef* mpuInitTypeDef, I2C_HandleTypeDef* i2cHandle)
{
    uint8_t ui8buffer;
    uint32_t i=0;
    //I2c Handle Typedef
    memcpy(&mpuhi2c, i2cHandle, sizeof(*i2cHandle));
//    MPU6050_InitTypedef mpuInit;

    //Read Who AM I Register
    MPU6050_Read(&ui8buffer, MPU6050_WHO_AM_I_REG, 1);
    if (ui8buffer != 0x68)
        return MPU6050_ID_ERROR;
    //Reset IMU
    MPU6050_Read(&ui8buffer, MPU6050_PWR_MGMT_1_REG, 1);
    ui8buffer |= 0x80;
    MPU6050_Write(&ui8buffer, MPU6050_PWR_MGMT_1_REG, 1);
    for(i = 0; i < 2400000; i++)
    {
    	asm("NOP");
    }

    ui8buffer = 0x08;           //Enable IMU
    MPU6050_Write(&ui8buffer, MPU6050_PWR_MGMT_1_REG, 1);
    //Setting Accelerometer full scale range
    ui8buffer = 0x00;
    ui8buffer |= mpuInitTypeDef->ui8AcceFullScale;
    MPU6050_Write(&ui8buffer, MPU6050_ACCEL_CONFIG_REG, 1);

    //Setting Gyroscope full scale range
    ui8buffer = 0x00;
    ui8buffer |= mpuInitTypeDef->ui8GyroFullScale;
    MPU6050_Write(&ui8buffer, MPU6050_GYRO_CONFIG_REG, 1);

    //Setting Digital Low Pass Filter
    ui8buffer = 0x00;
    ui8buffer |= mpuInitTypeDef->ui8DLPF;
    MPU6050_Write(&ui8buffer, MPU6050_CONFIG_REG, 1);

    //Setting sensitivity for Accelerometer
    switch (mpuInitTypeDef->ui8AcceFullScale)
    {
        case MPU6050_ACCE_FULLSCALE_2G:
            Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_2G;
            break;
        case MPU6050_ACCE_FULLSCALE_4G:
            Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_4G;
            break;
        case MPU6050_ACCE_FULLSCALE_8G:
            Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_8G;
            break;
        case MPU6050_ACCE_FULLSCALE_16G:
            Mpu6050_AcceSensi = MPU6050_ACCE_SENSI_16G;
            break;
        default:
            return MPU6050_ACCE_FULLSCALED_ERROR;
    }

    switch (mpuInitTypeDef->ui8GyroFullScale)
    {
        case MPU6050_GYRO_FULLSCALE_250DPS:
            Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_250DPS;
            break;
        case MPU6050_GYRO_FULLSCALE_500DPS:
            Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_500DPS;
            break;
        case MPU6050_GYRO_FULLSCALE_1000DPS:
            Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_1000DPS;
            break;
        case MPU6050_GYRO_FULLSCALE_2000DPS:
            Mpu6050_GyroSensi = MPU6050_GYRO_SENSI_2000DPS;
            break;
        default:
            return MPU6050_GYRO_FULLSCALED_ERROR;
    }
    return MPU6050_OK;

}

//4 . MPU6050 Accelerometer read data raw
MPU6050_AcceDataRaw MPU6050_AcceRead_Raw (void)
{
    uint8_t ui8Buffer[2];
    MPU6050_AcceDataRaw acRaw;

    // Read X value
    MPU6050_Read(ui8Buffer, MPU6050_ACCEL_XOUT_H_REG, 2);
    acRaw.x = ui8Buffer[0] << 8 | ui8Buffer[1];
    // Read Y value
    MPU6050_Read(ui8Buffer, MPU6050_ACCEL_YOUT_H_REG, 2);
    acRaw.y = ui8Buffer[0] << 8 | ui8Buffer[1];
    // Read Z value
    MPU6050_Read(ui8Buffer, MPU6050_ACCEL_ZOUT_H_REG, 2);
    acRaw.z = ui8Buffer[0] << 8 | ui8Buffer[1];
    return acRaw;
    }

//5. MPU6050 Accelerometer read data scaled
MPU6050_AcceDataScaled MPU6050_AcceRead_Scaled (void)
{
    MPU6050_AcceDataRaw mpuRaw;
    MPU6050_AcceDataScaled mpuScaled;
    mpuRaw = MPU6050_AcceRead_Raw();
    mpuScaled.x = mpuRaw.x/Mpu6050_AcceSensi;
    mpuScaled.y = mpuRaw.y/Mpu6050_AcceSensi;
    mpuScaled.z = mpuRaw.z/Mpu6050_AcceSensi;
    return mpuScaled;
    }

//6. MPU6050 Gyroscope read data raw
MPU6050_GyroDataRaw MPU6050_GyroRead_Raw (void)
{
    MPU6050_GyroDataRaw mpuRaw;
    uint8_t ui8Buffer[2];

    //Read X value
    MPU6050_Read(ui8Buffer, MPU6050_GYRO_XOUT_H_REG, 2);
    mpuRaw.x = ui8Buffer[0] << 8 | ui8Buffer [1];
    //Read Y value
    MPU6050_Read(ui8Buffer, MPU6050_GYRO_YOUT_H_REG, 2);
    mpuRaw.y = ui8Buffer[0] << 8 | ui8Buffer [1];
    //Read Z value
    MPU6050_Read(ui8Buffer, MPU6050_GYRO_ZOUT_H_REG, 2);
    mpuRaw.z = ui8Buffer[0] << 8 | ui8Buffer [1];

    return mpuRaw;
    }

//7. MPU6050 Gyroscope read data scaled
MPU6050_GyroDataScaled MPU6050_GyroRead_Scaled (void)
{
    MPU6050_GyroDataRaw mpuRaw;
    MPU6050_GyroDataScaled mpuScaled;
    mpuRaw = MPU6050_GyroRead_Raw();
    mpuScaled.x = mpuRaw.x/Mpu6050_GyroSensi;
    mpuScaled.y = mpuRaw.y/Mpu6050_GyroSensi;
    mpuScaled.z = mpuRaw.z/Mpu6050_GyroSensi;

    return mpuScaled;
    }

//8. MPU6050 Gyroscope calibration
void MPU6050_Calib (MPU6050_AcceAxis* mpuAcceAxis,MPU6050_GyroDataScaled * mpuGyroScaled, uint16_t ui16times)
{
	MPU6050_GyroDataScaled mpuScaled, mpuBuffer;
	MPU6050_AcceDataScaled mpuAcBuffer;
	MPU6050_AcceAxis mpuAxis;
	uint16_t i;
	mpuScaled.x = 0;
	mpuScaled.y = 0;
	mpuScaled.z = 0;
	mpuAxis.pitch = 0;
	mpuAxis.roll = 0;
	for(i = 0; i < ui16times; i++)
	{
		mpuBuffer = MPU6050_GyroRead_Scaled();
		mpuScaled.x += mpuBuffer.x;
		mpuScaled.y += mpuBuffer.y;
		mpuScaled.z += mpuBuffer.z;
		mpuAcBuffer = MPU6050_AcceRead_Scaled();
		mpuAxis.pitch += -180* atan2f(mpuAcBuffer.x, sqrt(mpuAcBuffer.y*mpuAcBuffer.y + mpuAcBuffer.z*mpuAcBuffer.z))/M_PI;
		mpuAxis.roll += 180* atan2f(mpuAcBuffer.y, sqrt(mpuAcBuffer.x*mpuAcBuffer.x + mpuAcBuffer.z*mpuAcBuffer.z))/M_PI;
		/* USER CODE BEGIN */
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		/* USER CODE END */
		HAL_Delay(50);
	}
	mpuGyroScaled->x = mpuScaled.x/ui16times;
	mpuGyroScaled->y = mpuScaled.y/ui16times;
	mpuGyroScaled->z = mpuScaled.z/ui16times;

	mpuAcceAxis->pitch = mpuAxis.pitch/ui16times;
	mpuAcceAxis->roll = mpuAxis.roll/ui16times;
	}
                                                                /** Copyright (C) 2021 - TTSang **/


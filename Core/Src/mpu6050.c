/*
 * mpu6050.c
 *
 *  Created on: Jul 23, 2024
 *      Author: kamil
 */
#include <stdbool.h>
#include <math.h>

#include "i2c.h"

#include "mpu6050.h"




static void mpu_write_reg(mpu6050_typedef *mpu, uint8_t reg, uint8_t val)
{
	HAL_I2C_Mem_Write(mpu->hi2c, mpu->i2c_address, reg, 1, &val, 1, HAL_MAX_DELAY);
}


static uint8_t mpu_read_reg(mpu6050_typedef *mpu, uint8_t reg)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(mpu->hi2c, mpu->i2c_address, reg, 1, &tmp, 1, HAL_MAX_DELAY);
	return tmp;
}


static void mpu_reset(mpu6050_typedef *mpu, uint8_t value)
{
	uint8_t tmp = mpu_read_reg(mpu, PWR_MGMT_1);
	tmp &= ~(1<<7);
	tmp |= ((value & 0x1) << 7);
	mpu_write_reg(mpu, PWR_MGMT_1, tmp);
}


static void mpu_sleep_mode(mpu6050_typedef *mpu, uint8_t value)
{
	uint8_t tmp = mpu_read_reg(mpu, PWR_MGMT_1);
	tmp &= ~(1<<6);
	tmp |= ((value & 0x1) << 6);
	mpu_write_reg(mpu, PWR_MGMT_1, tmp);
}


void set_gyro_scale(mpu6050_typedef *mpu, gyro_range_typedef range)
{
	uint8_t tmp = mpu_read_reg(mpu, GYRO_CONFIG);
	tmp &= ~(3 << 3);
	tmp |= (range & 0x3) << 3;
	mpu_write_reg(mpu, GYRO_CONFIG, tmp);

	switch (range){
	case range_250:
		mpu->gyro_scale = 0.007633;
		break;
	case range_500:
		mpu->gyro_scale = 0.015267;
		break;
	case range_1000:
		mpu->gyro_scale = 0.030487;
		break;
	case range_2000:
		mpu->gyro_scale = 0.060975;
		break;
	default:
		break;
	}
}


void set_accelerometer_scale(mpu6050_typedef *mpu, accelerometer_range_typedef range)
{
	uint8_t tmp = mpu_read_reg(mpu, ACCEL_CONFIG);
	tmp &= ~(3 << 3);
	tmp |= (range & 0x3) << 3;
	mpu_write_reg(mpu, ACCEL_CONFIG, tmp);

	switch (range)
	{
	case range_2g:
		mpu->acc_scale = 0.000061;
		break;
	case range_4g:
		mpu->acc_scale = 0.000122;
		break;
	case range_8g:
		mpu->acc_scale = 0.000244;
		break;
	case range_16g:
		mpu->acc_scale = 0.0004882;
		break;
	default:
		break;
	}
}


void mpu_low_pass_filter(mpu6050_typedef *mpu, low_pass_filter_typedef filter)
{
	uint8_t tmp = mpu_read_reg(mpu, CONFIG);
	tmp &= ~ 7;
	tmp |= filter & 0x7;
	mpu_write_reg(mpu, CONFIG, tmp);
}


HAL_StatusTypeDef mpu_who_am_i(mpu6050_typedef *mpu)
{
	uint8_t value = mpu_read_reg(mpu, WHO_AM_I);
	if (value == 0x68){
		return HAL_OK;
	}
	else{
		return HAL_ERROR;
	}
}

mpu6050_typedef mpu_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_address)
{
	mpu6050_typedef mpu;
	mpu.hi2c = hi2c;
	mpu.i2c_address = i2c_address;


	mpu_reset(&mpu, 1);

	HAL_Delay(1000);

	mpu_sleep_mode(&mpu, 0);

	set_gyro_scale(&mpu, range_250);
	set_accelerometer_scale(&mpu, range_2g);

	mpu.gx_bias = 0;
	mpu.gy_bias = 0;
	mpu.gz_bias = 0;
	mpu.lst_time_x_angle = HAL_GetTick();
	mpu.lst_time_y_angle = HAL_GetTick();


	return mpu;
}



void mpu_get_data(mpu6050_typedef *mpu)
{
	uint8_t data[14];
	HAL_I2C_Mem_Read(mpu->hi2c, mpu->i2c_address, ACCEL_XOUT_H, 1, data, 14, HAL_MAX_DELAY);

	mpu->ax = (int16_t)((int16_t)data[0] << 8 | data[1]) * mpu->acc_scale;
	mpu->ay = (int16_t)((int16_t)data[2] << 8 | data[3]) * mpu->acc_scale;
	mpu->az = (int16_t)((int16_t)data[4] << 8 | data[5]) * mpu->acc_scale;

	mpu->gx = ((int16_t)((int16_t)data[8] << 8 | data[9]) * mpu->gyro_scale) - mpu->gx_bias;
	mpu->gy = ((int16_t)((int16_t)data[10] << 8 | data[11]) * mpu->gyro_scale) - mpu->gy_bias;
	mpu->gz = ((int16_t)((int16_t)data[12] << 8 | data[13]) * mpu->gyro_scale) - mpu->gz_bias;
}


void mpu_gyro_calibration(mpu6050_typedef *mpu)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	uint8_t counter = 100;
	float gx_sum = 0;
	float gy_sum = 0;
	float gz_sum = 0;

	for(int i = 0; i < counter; i++)
	{
		mpu_get_data(mpu);
		if(fabs(mpu->gx) > 8 || fabs(mpu->gy) > 8 ||fabs(mpu->gz) > 8){
			i = 0;
			gx_sum = 0;
			gy_sum = 0;
			gz_sum = 0;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			HAL_Delay(1000);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			continue;
		}

		gx_sum += mpu->gx;
		gy_sum += mpu->gy;
		gz_sum += mpu->gz;

		HAL_Delay(4);
	}
	// += instead of = is used in case of second calibration (bias is added in mpu_get_data)
	mpu->gx_bias += gx_sum / counter;
	mpu->gy_bias += gy_sum / counter;
	mpu->gz_bias += gz_sum / counter;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}


float mpu_get_acc_x_angle(mpu6050_typedef *mpu)
{
	return atan2(mpu->ax, mpu->az) + M_PI/2;
}


float mpu_get_acc_y_angle(mpu6050_typedef *mpu)
{
	return atan2(mpu->ay, mpu->az);

}


void mpu_calc_x_angle(mpu6050_typedef *mpu)
{
	mpu_get_data(mpu);
	float delta = (HAL_GetTick() - mpu->lst_time_x_angle) / 1000.0;
	mpu->lst_time_x_angle = HAL_GetTick();
	if(delta > 0.01)
	{
		mpu->x_angle = mpu_get_acc_x_angle(mpu);
		mpu->lst_time_x_angle = HAL_GetTick();
	}
	else
	{
		//complementary filter
		float alpha = 0.005;
		mpu->x_angle = alpha * mpu_get_acc_x_angle(mpu) + (1 - alpha) * (mpu->x_angle + (-mpu->gy * DEG2RAD * delta));
	}
}


void mpu_calc_y_angle(mpu6050_typedef *mpu)
{
	mpu_get_data(mpu);
	float delta = (HAL_GetTick() - mpu->lst_time_y_angle) / 1000.0;
	if(delta > 0.05)
	{
		mpu->y_angle = mpu_get_acc_y_angle(mpu);
		mpu->lst_time_y_angle = HAL_GetTick();
	}
	else
	{
		// complementary filter
		float alpha = 0.01;
		mpu->y_angle = alpha * mpu_get_acc_y_angle(mpu) + (1 - alpha) * (mpu->y_angle + (mpu->gx * DEG2RAD * delta));
		mpu->lst_time_y_angle = HAL_GetTick();
	}
}






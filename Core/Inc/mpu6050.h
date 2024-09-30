/*
 * mpu6050.h
 *
 *  Created on: Jul 23, 2024
 *      Author: kamil
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


// config
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C

// accelerometer data registers
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define	ACCEL_ZOUT_L 0x40

// temperature registers
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

// gyro data registers
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define	GYRO_ZOUT_L 0x48

// power modes
#define PWR_MGMT_1 0x6B

#define WHO_AM_I 0x75   // default value 0x68

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

//#define MPU6050_ADDRESS 0xD0



typedef struct{
	//config
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;
	float gyro_scale, acc_scale;
	float gx_bias, gy_bias, gz_bias;
	//data
	float ax, ay, az, gx, gy, gz;
	float x_angle, y_angle;
	unsigned long int lst_time_x_angle, lst_time_y_angle;
}mpu6050_typedef;


typedef enum{
	Acc260Hz_Gyro256Hz = 0,
	Acc184Hz_Gyro188Hz,
	Acc94Hz_Gyro98Hz,
	Acc44Hz_Gyro42Hz,
	Acc21Hz_Gyro20Hz,
	Acc10Hz_Gyro10Hz,
	Acc5Hz_Gyro5Hz
}low_pass_filter_typedef;


typedef enum{
	range_250 = 0,
	range_500,
	range_1000,
	range_2000
}gyro_range_typedef;


typedef enum{
	range_2g = 0,
	range_4g,
	range_8g,
	range_16g
}accelerometer_range_typedef;






void set_gyro_scale(mpu6050_typedef *mpu, gyro_range_typedef range);

void set_accelerometer_scale(mpu6050_typedef *mpu, accelerometer_range_typedef range);

void mpu_low_pass_filter(mpu6050_typedef *mpu, low_pass_filter_typedef filter);

HAL_StatusTypeDef mpu_who_am_i(mpu6050_typedef *mpu);

mpu6050_typedef mpu_init(I2C_HandleTypeDef *hi2c, uint8_t i2c_address);

void mpu_gyro_calibration(mpu6050_typedef *mpu);

void mpu_get_data(mpu6050_typedef *mpu);

float mpu_get_acc_x_angle(mpu6050_typedef *mpu);

float mpu_get_acc_y_angle(mpu6050_typedef *mpu);

void mpu_calc_x_angle(mpu6050_typedef *mpu);

void mpu_calc_y_angle(mpu6050_typedef *mpu);




#endif /* INC_MPU6050_H_ */


/*
 * regulator.c
 *
 *  Created on: Feb 5, 2025
 *      Author: kamil
 */


#include <robot.h>


extern uart_interface_typedef uart_interface;

uint8_t I2C_DMA_buffer[12];


//temp
extern int32_t step_counter_temp;
extern int32_t time_temp;
extern int8_t speed_temp;


robot_typedef robot = {
		.position = 0,
		.angle = 0,
		.battery_voltage = 0,
		.set_position = 0,
		.set_angle = 0,
		.control_on = 1,
		.send_log = 1,
		.encoder_angle = 0
};




int16_t counter;


void control_loop(){

	if(counter >= 500){
		stepper_set_speed(robot.stepper1, -robot.stepper1->speed);
		counter = 0;
	}
	else{
		counter++;
	}





	if(robot.send_log){
		uint8_t buffer[BUFFER_SIZE_TX];
		uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%ld, %ld, %d, %ld\r\n", robot.encoder_angle, step_counter_temp, speed_temp, time_temp);
		uart_send(&uart_interface, buffer, size, 0);
	}
}



void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c == robot.mpu->hi2c){
		robot.mpu->ax = (int16_t)((int16_t)I2C_DMA_buffer[0] << 8 | I2C_DMA_buffer[1]) * robot.mpu->acc_scale;
		robot.mpu->az = (int16_t)((int16_t)I2C_DMA_buffer[4] << 8 | I2C_DMA_buffer[5]) * robot.mpu->acc_scale;
		robot.mpu->gy = ((int16_t)((int16_t)I2C_DMA_buffer[10] << 8 | I2C_DMA_buffer[11]) * robot.mpu->gyro_scale) - robot.mpu->gy_bias;
		robot.mpu->lst_update_x_angle = HAL_GetTick();
	}
	else if (hi2c == &hi2c2){  // encoder
		calculate_encoder_angle();
	}

	if(robot.control_on){
		control_loop();
	}
}




/*
 * robot.c
 *
 *  Created on: Jul 23, 2025
 *      Author: kamil
 */


#include "stm32f1xx_hal.h"
#include "main.h"

#include "robot.h"
#include "helpers.h"
#include "mpu6050.h"
#include "stepper_lib.h"
#include "communication_interface.h"



extern volatile stepper_typedef stepper1;
extern volatile stepper_typedef stepper2;
extern mpu6050_typedef mpu;
extern communication_interface_t communication_interface;


robot_t robot;



void robot_init(){
	robot.data_ready = 0;
	robot.robot_enable = 1;
	robot.control_enable = 0;

	robot.K1 = 2.013;
	robot.K2 = 0.2526;
	robot.K3 = -1.0544;
	robot.K4 = -0.7421;
	robot.K5 = -0.1536;

	robot.set_angle = -0.06;
	robot.set_pos = 0;
	robot.set_speed = 0;

	robot.d_angle_filter = filter_init(0.8);
	robot.d_pos_filter = filter_init(0.8);

}


void control_step(){
	robot.d_angle = (mpu.x_angle - robot.prev_angle) / TIME_DELTA;
	robot.d_angle = filter(&robot.d_angle_filter, robot.d_angle);
	robot.pos = stepper1.step_counter * STEP2METERS;
	robot.d_pos = filter(&robot.d_pos_filter, (robot.pos - robot.prev_pos) / TIME_DELTA);
	robot.pos_error = robot.set_pos - robot.pos;

	robot.pos_int -= robot.pos_error * TIME_DELTA;


	saturation(-0.3, 0.3, &robot.pos_error);

	robot.speed_delta = (robot.set_angle - mpu.x_angle) * robot.K1
			- robot.d_angle * robot.K2
			+ robot.pos_error * robot.K3
			+ (robot.set_speed - robot.d_pos) * robot.K4
			- robot.pos_int * robot.K5;

	saturation(-0.35, 0.35, &robot.speed_delta);

	robot.speed += robot.speed_delta * METERS2RAD;

	stepper_set_speed(&stepper1, robot.speed);
	stepper_set_speed(&stepper2, robot.speed);

	robot.data_ready = 0;

	//reset prev values
	robot.prev_angle = mpu.x_angle;
	robot.prev_pos = robot.pos;
}


void robot_start(){
	stepper_enable(&stepper1, 1);
	stepper_enable(&stepper2, 1);
	stepper1.step_counter = 0;
	stepper2.step_counter = 0;

	robot.control_enable = 1;
	robot.speed = 0;
	robot.prev_angle = 0;
	robot.prev_pos = 0;
	robot.pos_int = 0;

	robot.set_pos = 0;
	robot.set_speed = 0;
	robot.pos_error = 0;

	reset_filter(&robot.d_angle_filter);
	reset_filter(&robot.d_pos_filter);

}


void robot_stop(){
	stepper_enable(&stepper1, 0);
	stepper_enable(&stepper2, 0);
	robot.control_enable = 0;
}



void battery_voltage_update(uint16_t voltage){
	robot.battery_voltage = voltage * 0.0045934;
		if(robot.battery_voltage < 11.1){
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			if(robot.battery_voltage < 10.4){
				robot.robot_enable = 0;
			}
		}
		else{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			robot.robot_enable = 1;
		}
}




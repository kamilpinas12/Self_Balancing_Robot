/*
 * regulator.h
 *
 *  Created on: Feb 5, 2025
 *      Author: kamil
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_


#include <stdio.h>
#include <stdbool.h>


//HAL
#include "dma.h"
#include "i2c.h"


// private
#include "mpu6050.h"
#include "stepper_lib.h"
#include "pid_lib.h"
#include "uart_interface.h"
#include "helpers.h"
#include "low_pass_filter.h"



typedef struct {
	mpu6050_typedef * mpu;
	volatile stepper_typedef* stepper1;
	volatile stepper_typedef* stepper2;

	//state
	int32_t pos;
	float angle; // angle of the robot (rotation) not angle from mpu6050
	float battery_voltage;
	int32_t encoder_angle;
	int32_t prev_pos;

	//control
	volatile int32_t set_position;
	volatile float set_angle;

	//regulator
	pid_typedef* angle_pid;
	pid_typedef* pos_pid;
	pid_typedef* target_speed_pid;


	//flags
	bool control_on;
	bool send_data;


}robot_typedef;






//void robot_init(robot_typedef* robot);


void control_loop();

void robot_start();

void robot_stop();











#endif /* INC_ROBOT_H_ */

/*
 * regulator.h
 *
 *  Created on: Feb 5, 2025
 *      Author: kamil
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_



//HAL
#include"dma.h"


// private
#include "mpu6050.h"
#include "stepper_lib.h"
#include "pid_lib.h"


typedef struct {
	volatile mpu6050_typedef* mpu;
	volatile stepper_typedef* stepper1;
	volatile stepper_typedef* stepper2;

	//state
	int32_t position;
	float angle;
	float battery_voltage;

	//control
	volatile int32_t set_position;
	volatile float set_angle;

	//regulator
	pid_typedef* angle_pid;
	pid_typedef* pos_pid;




}robot_typedef;






void robot_init(robot_typedef* robot);

















#endif /* INC_ROBOT_H_ */

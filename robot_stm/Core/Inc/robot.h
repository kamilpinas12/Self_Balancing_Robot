/*
 * robot.h
 *
 *  Created on: Jul 23, 2025
 *      Author: kamil
 */

#ifndef INC_ROBOT_H_
#define INC_ROBOT_H_


#include<stdbool.h>

#include"low_pass_filter.h"



typedef struct {
	//flags
	uint8_t data_ready;
	uint8_t robot_enable;
	uint8_t control_enable;

	//state
	float speed;
	float d_angle;
	float d_pos;
	float pos;
	float pos_int; // position integral
	float speed_delta;

	float pos_error;

	// set point
	float set_angle;
	float set_pos;
	float set_speed;

	//prev values
	float prev_angle;
	float prev_pos;



	//parameters
	float K1;
	float K2;
	float K3;
	float K4;
	float K5;

	//state
	float battery_voltage;

	//filters
	filter_typedef d_angle_filter;
	filter_typedef speed_filter;

}robot_t;




void robot_init();

void robot_start();

void robot_stop();

void control_step();

void battery_voltage_update(uint16_t voltage);


#endif /* INC_ROBOT_H_ */

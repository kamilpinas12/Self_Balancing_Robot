/*
 * helpers.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#include <stdlib.h>


#include "helpers.h"
#include "robot.h"

extern robot_typedef robot;
extern uint8_t encoder_data_buffer[2];



void saturation(float min, float max, float* val){
	if(*val > max) *val = max;
	if(*val < min) *val = min;
}



//uint16_t get_angle_error(int16_t angle, int16_t desired_angle){
//	int16_t error = desired_angle - angle;
//
//	if(abs(error) > 180){
//		if (error > 0) error -= 360;
//		else error += 360;
//	}
//	return error;
//}
//
//
//void calculate_encoder_angle(){
//	static int16_t prev_angle;
//
//	int16_t new_angle = ((uint16_t)encoder_data_buffer[0] << 8) | (uint16_t)encoder_data_buffer[1];
//
//	if(prev_angle > 3600 && new_angle < 500){ //overflow
//		robot.encoder_angle += 4095 - prev_angle + new_angle;
//	}
//	else if(prev_angle < 500 && new_angle > 3600){ //underflow
//		robot.encoder_angle += -prev_angle - (4095 - new_angle);
//	}
//	else{
//		robot.encoder_angle += new_angle - prev_angle;
//	}
//
//	prev_angle = new_angle;
//
//}










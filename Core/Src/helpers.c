/*
 * helpers.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#include <stdlib.h>


#include "helpers.h"




void saturation(float min, float max, float* val){
	if(*val > max) *val = max;
	if(*val < min) *val = min;
}



uint16_t get_angle_error(uint16_t angle, uint16_t desired_angle){
	int error = desired_angle - angle;

	if(abs(error) > 180){
		if (error > 0) error -= 360;
		else error += 360;
	}
	return error;
}










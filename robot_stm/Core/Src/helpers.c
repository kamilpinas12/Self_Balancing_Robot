/*
 * helpers.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#include <stdlib.h>

#include "stm32f1xx_hal.h"

#include "helpers.h"



volatile uint8_t encoder_data_buffer[2];
volatile float encoder_angle;




void calculate_encoder_angle(){
	static int16_t prev_angle;

	int16_t new_angle = ((uint16_t)encoder_data_buffer[0] << 8) | (uint16_t)encoder_data_buffer[1];

	if(prev_angle > 3600 && new_angle < 500){ //overflow
		encoder_angle += 4095 - prev_angle + new_angle;
	}
	else if(prev_angle < 500 && new_angle > 3600){ //underflow
		encoder_angle += -prev_angle - (4095 - new_angle);
	}
	else{
		encoder_angle += new_angle - prev_angle;
	}

	prev_angle = new_angle;

}



void saturation(float min, float max, float* val){
	if(*val > max) *val = max;
	if(*val < min) *val = min;
}





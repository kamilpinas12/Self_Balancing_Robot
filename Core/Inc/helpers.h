/*
 * helpers.h
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include <stdint.h>


void saturation(float min, float max, float* val);

uint16_t get_angle_error(uint16_t angle, uint16_t desired_angle);

void calculate_encoder_angle();



#endif /* INC_HELPERS_H_ */

/*
 * helpers.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */


#include "helpers.h"


void saturation(float min, float max, float* val){
	if(*val > max) *val = max;
	if(*val < min) *val = min;
}

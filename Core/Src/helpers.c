/*
 * helpers.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#include<math.h>


#include "helpers.h"

#define RET_FUN_SATURATION 100.0f


void saturation(float min, float max, float* val){
	if(*val > max) *val = max;
	if(*val < min) *val = min;
}


double return_function(int32_t arg){
	if(arg > RET_FUN_SATURATION || arg < -RET_FUN_SATURATION){
		return 1;
	}
	double value = pow((1/RET_FUN_SATURATION) * arg, 2);

	return value;
}











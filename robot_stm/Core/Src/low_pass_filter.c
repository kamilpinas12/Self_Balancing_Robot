/*
 * low_pass_filter.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#include "low_pass_filter.h"


filter_typedef filter_init(float alpha){
	filter_typedef f = {alpha, 0, 1};
	return f;
}


float filter(filter_typedef* filter, float new_value){
	if(filter->reset){
		filter->reset = 0;
		filter->prev_value = new_value;
		return new_value;
	}
	else{
		float filtered_value = (filter->alpha) * new_value + (1 - filter->alpha) * filter->prev_value;
		filter->prev_value = filtered_value;
		return filtered_value;
	}
}


void reset_filter(filter_typedef* filter){
	filter->reset = 1;
}





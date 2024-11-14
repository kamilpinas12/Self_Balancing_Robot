/*
 * low_pass_filter.h
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#ifndef INC_LOW_PASS_FILTER_H_
#define INC_LOW_PASS_FILTER_H_

#include<stdbool.h>

#include "stm32f1xx_hal.h"


typedef struct{
	float alpha;
	float prev_value;
	bool reset;

}filter_typedef;



float get_value(filter_typedef* filter, float new_value);



#endif /* INC_LOW_PASS_FILTER_H_ */

/*
 * stepper_lib.c
 *
 *  Created on: Aug 4, 2024
 *      Author: kamil
 */



#include "stepper_lib.h"
#include "helpers.h"


#define SEC2uSEC 1e6f


extern volatile uint8_t spin_duration_ms;


static void set_dir(stepper_typedef *stepper, int8_t dir){
	if(stepper->dir_polarity * dir == 1){
		HAL_GPIO_WritePin(stepper->DIR_Port, stepper->DIR_Pin, GPIO_PIN_SET);
		stepper->dir = stepper->dir_polarity;
	}
	else{
		HAL_GPIO_WritePin(stepper->DIR_Port, stepper->DIR_Pin, GPIO_PIN_RESET);
		stepper->dir = -stepper->dir_polarity;
	}
}



//dir pin polarity : 1 or -1
void stepper_init(stepper_typedef *stepper, TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
		GPIO_TypeDef *DIR_Port, uint16_t DIR_Pin, unsigned int max_speed, int8_t dir_polarity)
{
	stepper->htim = htim;
	stepper->Channel = Channel;

	stepper->EN_Port = EN_Port;
	stepper->EN_Pin = EN_Pin;
	stepper->DIR_Port = DIR_Port;
	stepper->DIR_Pin = DIR_Pin;

	stepper->step_counter = 0;
	stepper->on_off = 0;
	stepper-> new_counter = 65000;

	stepper->dir_polarity = dir_polarity;
	stepper->max_speed = max_speed;

	set_dir(stepper, 1);
	stepper_enable(stepper, 0);
	stepper_set_speed(stepper, 0);
	stepper->speed = 0;


	// start timer
	HAL_TIM_Base_Start_IT(htim);
	HAL_TIM_PWM_Start(htim, Channel);
}



void stepper_enable(stepper_typedef *stepper, bool en)
{
	if(en){
		HAL_GPIO_WritePin(stepper->EN_Port, stepper->EN_Pin, GPIO_PIN_RESET);
		stepper->enable = 1;
	}
	else{
		HAL_GPIO_WritePin(stepper->EN_Port, stepper->EN_Pin, GPIO_PIN_SET);
		stepper_set_speed(stepper, 0);
		stepper->enable = 0;
		stepper->on_off = 0;
		stepper->speed = 0;
	}
}



void stepper_set_speed(stepper_typedef *stepper, float speed)
{
	saturation(-stepper->max_speed, stepper->max_speed, &speed);
	stepper->speed = speed;
	if(fabs(speed) < 0.0628) {
		stepper->on_off = 0;
		__HAL_TIM_SET_COMPARE(stepper->htim, stepper->Channel, 0);
		__HAL_TIM_SET_AUTORELOAD(stepper->htim, 1000);
		return;
	}
	else{
		stepper->on_off = 1;
		uint16_t counter = (2.0*3.14158/(fabs(speed)*1600.0)) * SEC2uSEC;
		//uint16_t counter = 3926.
		stepper->new_counter = counter;

		if(speed > 0) set_dir(stepper, 1);
		else set_dir(stepper, -1);
	}
}


void stepper_update(stepper_typedef *stepper)
{
	if(stepper->on_off){
		stepper->step_counter += stepper->dir;
		__HAL_TIM_SET_AUTORELOAD(stepper->htim, stepper->new_counter);
		__HAL_TIM_SET_COMPARE(stepper->htim, stepper->Channel, 10);
	}
}



/*
 * stepper_lib.h
 *
 *  Created on: Aug 4, 2024
 *      Author: kamil
 */

#ifndef INC_STEPPER_LIB_H_
#define INC_STEPPER_LIB_H_

#include <stdlib.h>
#include <stdbool.h>


#include "stm32f1xx_hal.h"
#include "tim.h"


#define NUMBER_OF_STEPPER_MOTORS 2



volatile typedef struct {

	TIM_HandleTypeDef *htim;
	uint32_t Channel;

	//pin config
	GPIO_TypeDef *EN_Port;
	uint16_t EN_Pin;

	GPIO_TypeDef *DIR_Port;
	uint16_t DIR_Pin;

	int max_speed; // steps per second

	int step_counter;
	bool on_off;
	int new_counter;


	bool enable;
	int8_t dir; //1 or -1

	int dir_polarity;
}stepper_typedef;



void stepper_init(stepper_typedef *stepper, TIM_HandleTypeDef *htim, uint32_t Channel, GPIO_TypeDef *EN_Port, uint16_t EN_Pin,
		GPIO_TypeDef *DIR_Port, uint16_t DIR_Pin, unsigned int max_speed, int dir_polarity);

void stepper_enable(stepper_typedef *stepper, bool en);

void stepper_set_speed(stepper_typedef *stepper, float speed);

void stepper_update(stepper_typedef *stepper);


#endif /* INC_STEPPER_LIB_H_ */

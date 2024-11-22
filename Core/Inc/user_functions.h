/*
 * user_function.h
 *
 *  Created on: Nov 19, 2024
 *      Author: kamil
 */

#ifndef INC_USER_FUNCTIONS_H_
#define INC_USER_FUNCTIONS_H_

#include<stdio.h>
#include<string.h>
#include<stdlib.h>

#include "stm32f1xx_hal.h"
#include "main.h"

#include<uart_interface.h>



//USER FUNCTIONS DECLARATION
void led(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void comunication_test(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void set_position(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void rotate(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);



#endif /* INC_USER_FUNCTIONS_H_ */

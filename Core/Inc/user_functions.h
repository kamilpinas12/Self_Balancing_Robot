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
#include<stdbool.h>

#include "stm32f1xx_hal.h"
#include "main.h"

#include "uart_interface.h"
#include "pid_lib.h"
#include "stepper_lib.h"
#include "robot.h"




//USER FUNCTIONS DECLARATION
void led(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void comunication_test(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void motor_test(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void motor_enable(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void controler_start(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void controler_stop(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void send_data(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void battery_voltage(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);


void set_position(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);
//
//void set_angle_fun(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);
//
void rotate_deg(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void get_angle(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);

void move(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]);


#endif /* INC_USER_FUNCTIONS_H_ */

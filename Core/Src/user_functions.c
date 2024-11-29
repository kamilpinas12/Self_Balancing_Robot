/*
 * user_function.c
 *
 *  Created on: Nov 19, 2024
 *      Author: kamil
 */


#include "user_functions.h"



extern uart_interface_typedef uart_interface;
extern int32_t set_pos;

extern uint8_t spin_duration_ms;
extern int8_t spin_value;

extern pid_typedef pos_pid;




void led(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	if(strcmp(args[0], "1") == 0){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	if(strcmp(args[0], "0") == 0){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
}

void comunication_test(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	printf("Czesc ;)\n");
}




void set_position(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){

	if(strcmp(args[0], "0") == 0){
		set_pos = 0;
	}
	else{
		int32_t val = atoi(args[0]);
		if(val == 0) return;
		set_pos = val;
	}
}



void rotate(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	spin_duration_ms = atoi(args[0]);
	spin_value = atoi(args[1]);
}



void change_pid(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	pos_pid.kp = atof(args[0]);
	pos_pid.ki = atof(args[1]);
	pos_pid.kd = atof(args[2]);


}
















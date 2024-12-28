/*
 * user_function.c
 *
 *  Created on: Nov 19, 2024
 *      Author: kamil
 */


#include "user_functions.h"


#define CM_TO_STEP 75.117
#define STEP_TO_CM 0.0133125




extern uart_interface_typedef uart_interface;
extern int32_t set_pos;

extern int16_t spin_duration_ms;
extern int8_t spin_value;

extern pid_typedef pos_pid;
extern float set_angle;





void led(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	if(strcmp(args[0], "1") == 0){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	}
	if(strcmp(args[0], "0") == 0){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
}

void comunication_test(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	uint8_t buffer[BUFFER_SIZE_TX];
	uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "----------------Czesc :)--------------\n");
	uart_send(&uart_interface, buffer, size, 1);

}


void set_position(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){

	if(strcmp(args[0], "0") == 0){
		set_pos = 0;
	}
	else{
		int32_t val = atoi(args[0]);
		if(val == 0) return;
		set_pos = val * CM_TO_STEP;
	}
}



void set_angle_fun(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	spin_duration_ms = atoi(args[0]);
	spin_value = atoi(args[1]);
}


void rotate_deg(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){

}















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

extern robot_typedef robot;




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
	uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "Command received!!!\n");
	uart_send(&uart_interface, buffer, size, 1);

}


void motor_test(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	float speed = atof(args[1]);
	if(strcmp(args[0], "1") == 0){
		stepper_set_speed(robot.stepper1, speed);
	}
	else if(strcmp(args[0], "2") == 0){
		stepper_set_speed(robot.stepper2, speed);
	}
	else{
		uint8_t buffer[BUFFER_SIZE_TX];
		uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "Select motor '1' or '2'\n");
		uart_send(&uart_interface, buffer, size, 1);
	}
}


void motor_enable(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	bool enable = 0;
	if(strcmp(args[0], "1") == 0) enable = 1;
	else if (strcmp(args[0], "0") == 0) enable = 0;
	else{
		uint8_t buffer[BUFFER_SIZE_TX];
		uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "enable must be '0' or '1'\n");
		uart_send(&uart_interface, buffer, size, 1);
	}
	stepper_enable(robot.stepper1, enable);
	stepper_enable(robot.stepper2, enable);

}



//void set_position(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
//
//	if(strcmp(args[0], "0") == 0){
//		set_pos = 0;
//	}
//	else{
//		int32_t val = atoi(args[0]);
//		if(val == 0) return;
//		set_pos = val * CM_TO_STEP;
//	}
//}
//
//
//
//void set_angle_fun(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
//	spin_duration_ms = atoi(args[0]);
//	spin_value = atoi(args[1]);
//}
//
//
//void rotate_deg(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
//	set_angle = atoi(args[0]) % 360;
//}
//
//













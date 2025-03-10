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
	if(robot.control_on) return;
	float speed = atof(args[1]);
	if(strcmp(args[0], "1") == 0){
		stepper_set_speed(robot.stepper1, speed);
	}
	else if(strcmp(args[0], "2") == 0){
		stepper_set_speed(robot.stepper2, speed);
	}
	else if(strcmp(args[0], "0") == 0){
		stepper_set_speed(robot.stepper1, speed);
		stepper_set_speed(robot.stepper2, speed);
	}
	else{
		uint8_t buffer[BUFFER_SIZE_TX];
		uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "Select motor '1' or '2' or both '0'\n");
		uart_send(&uart_interface, buffer, size, 1);
	}
}


void motor_enable(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	if(robot.control_on) return;
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


void controler_start(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	robot.stop = 0;
}


void controler_stop(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	robot.stop = 1;
	robot_stop();
}


void send_data(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	if(strcmp(args[0], "0") == 0){
		robot.send_data = 0;
	}
	else if(strcmp(args[0], "1") == 0){
		robot.send_data = 1;
	}

}


void battery_voltage(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	uint8_t buffer[BUFFER_SIZE_TX];
	float b = robot.battery_voltage * 50 -530;
	uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "Battery voltage: %.2f [V]  (%.1f%%)\r\n", robot.battery_voltage, b);
	uart_send(&uart_interface, buffer, size, 1);
}


void set_position(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){

	if(strcmp(args[0], "0") == 0){
		robot.target_pos = 0;
	}
	else{
		int32_t val = atoi(args[0]);
		if(val == 0) return;
		robot.target_pos = val * CM_TO_STEP;
	}
}


//
//void set_angle_fun(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
//	spin_duration_ms = atoi(args[0]);
//	spin_value = atoi(args[1]);
//}


void rotate_deg(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	robot.set_angle += atoi(args[0]) % 360;
}



void get_angle(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	uint8_t buffer[BUFFER_SIZE_TX];
	uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "angle: %.3f rad\r\n", robot.mpu->x_angle);
	uart_send(&uart_interface, buffer, size, 1);
}


void move(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	robot.target_pos += atoi(args[0])* CM_TO_STEP;
}


void get_pos(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){
	uint8_t buffer[BUFFER_SIZE_TX];
	uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "pos: %.3f, angle: %.3f\r\n", robot.pos * STEP_TO_CM, robot.angle);
	uart_send(&uart_interface, buffer, size, 1);
}





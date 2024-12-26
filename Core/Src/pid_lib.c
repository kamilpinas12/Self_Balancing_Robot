/*
 * pid_lib.c
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */



#include<stdbool.h>


#include "pid_lib.h"




pid_typedef pid_init(float kp, float ki, float kd){
	pid_typedef pid = {kp, ki, kd, 0, 0, 0, 0, 1};
	return pid;
}


void pid_reset(pid_typedef* pid){
	pid->i = 0;
	pid->prev_error = 0;
	pid->prev_measurement = 0;
}


void pid_step(pid_typedef* pid){
	pid->prev_error = pid->error;
	pid->prev_measurement = pid->measurement;
}





/*
 * pid_lib.h
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#ifndef INC_PID_LIB_H_
#define INC_PID_LIB_H_



typedef struct{
	float kp;
	float ki;
	float kd;

	float i;

	float error;
	float prev_error;

	float measurement;
	float prev_measurement;

}pid_typedef;


/*
 * instruction:
 * - use pid_init to initialize structure
 * - assign error and measurement to structure variables error and measurement (measurement variable can be use for derivative part)
 * - structure stores integral part
 * - after one step run pid_step it will assign error to prev_error and measurement to prev_measurement
 * - to reset integral and prev_error and prev_measurement run pid_reset
 */



pid_typedef pid_init(float kp, float ki, float kd);

void pid_reset(pid_typedef* pid);

void pid_step(pid_typedef* pid);


#endif /* INC_PID_LIB_H_ */

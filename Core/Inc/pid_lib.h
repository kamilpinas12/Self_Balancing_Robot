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

	bool reset;

}pid_typedef;


/*
 * instruction:
 * - use pid_init to initialize structure
 * - assign error value to pid.error
 * - to get prev_error use get_prev_error (it is used to return pid.error if function is used after reset)
 * - at the end of the loop assign pid.error to pid.prev_error
 * - to restart pid (set integral to 0 and pid_reset to 1)
 *
 */


pid_typedef pid_init(float kp, float ki, float kd);

void pid_reset(pid_typedef* pid);

float get_prev_error(pid_typedef* pid);


#endif /* INC_PID_LIB_H_ */

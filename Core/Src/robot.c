/*
 * regulator.c
 *
 *  Created on: Feb 5, 2025
 *      Author: kamil
 */


#include <robot.h>


extern uart_interface_typedef uart_interface;

uint8_t I2C_DMA_buffer[12];

filter_typedef pos_filter;
filter_typedef speed_filter;
filter_typedef target_angle_filter;






robot_typedef robot = {
		.pos = 0,
		.angle = 0,
		.battery_voltage = 0,
		.set_position = 0,
		.set_angle = 0,
		.control_on = 0,
		.send_data = 0,
		.encoder_angle = 0,
		.prev_pos = 0
};





void robot_start(){  // init function called before start of control loop

	stepper_enable(robot.stepper1, 1);
	stepper_enable(robot.stepper2, 1);
	stepper_set_speed(robot.stepper1, 0);
	stepper_set_speed(robot.stepper2, 0);

	robot.stepper1->step_counter = 0;
	robot.stepper2->step_counter = 0;


	//reset robot position and angle
	robot.set_position = 0;
	robot.set_angle = 0;
	robot.pos = 0;
	robot.angle = 0;
	robot.prev_pos = 0;



	// regulators reset
	pid_reset(robot.pos_pid);
	pid_reset(robot.angle_pid);
	pid_reset(robot.target_speed_pid);

	reset_filter(&pos_filter);
	reset_filter(&speed_filter);
	reset_filter(&target_angle_filter);


	robot.control_on = 1;
}


void robot_stop(){  // fuction called afterr control loop if for example robot fall ( stop the motors,  reset regulators)

	stepper_enable(robot.stepper1, 0);
	stepper_enable(robot.stepper2, 0);
	stepper_set_speed(robot.stepper1, 0);
	stepper_set_speed(robot.stepper2, 0);

	robot.control_on = 0;
}



float time_delta_ms = 4;




// log array
//#define SIZE 1000
//static float angle_log[SIZE];
//static float pid_log[SIZE];
//static uint16_t counter;
//

void control_loop(){

				// target speed PID
	robot.pos = (robot.stepper2->step_counter + robot.stepper1->step_counter)/2;
	robot.target_speed_pid->measurement = robot.pos;
	float error = robot.set_position - robot.pos;
	saturation(-2500, 2500, &error);
	robot.target_speed_pid->error = filter(&pos_filter, error);
	float speed = filter(&speed_filter, (robot.target_speed_pid->measurement - robot.target_speed_pid->prev_measurement) / time_delta_ms);


	// P
	float p = robot.target_speed_pid->error * robot.target_speed_pid->kp;

	// D
	float d = robot.target_speed_pid->kd * (-speed);

	float target_speed = p + d;
	saturation(-40, 40, &target_speed);


				// pos PID

	robot.pos_pid->error = target_speed - speed;
	robot.pos_pid->measurement = speed;

	// pos P
	float p_pos = robot.pos_pid->error * robot.pos_pid->kp;
	//
	// pos I
	robot.pos_pid->i += 0.5 * (robot.pos_pid->error + robot.pos_pid->prev_error) * robot.pos_pid->ki * time_delta_ms;
	saturation(-0.55, 0.55, &robot.pos_pid->i);

	// pos D
	float d_pos = robot.pos_pid->kd * (robot.pos_pid->measurement - robot.pos_pid->prev_measurement) / time_delta_ms;
	saturation(-0.15, 0.15, &d_pos);


	float target_angle = p_pos + robot.pos_pid->i + d_pos;
	saturation(-0.2, 0.2, &target_angle);


				// angle PID
	float target_angle_filtered = -filter(&target_angle_filter, target_angle);
	robot.angle_pid->error = target_angle_filtered -robot.mpu->x_angle - 0.065;
	robot.angle_pid->measurement = -robot.mpu->x_angle;


	// P angle
	float angle_p = robot.angle_pid->error * robot.angle_pid->kp;

	// I angle
	robot.angle_pid->i += ((robot.angle_pid->error + robot.angle_pid->prev_error) * time_delta_ms * robot.angle_pid->ki) / 2;
	saturation(-60, 60, &(robot.angle_pid->i));

	// D angle
	float angle_d = robot.angle_pid->kd * (robot.angle_pid->measurement - robot.angle_pid->prev_measurement)/time_delta_ms;

	float pid = angle_p + robot.angle_pid->i + angle_d;


	// rotation error
	float robot_angle = (360.0 * 20)/(2*3.14*15.5 * 1600.0) * (robot.stepper2->step_counter - robot.stepper1->step_counter);

	float rotation_speed = (robot.set_angle - robot_angle) * 5;
	saturation(-50, 50, &rotation_speed);


	stepper_set_speed(robot.stepper1, pid - rotation_speed);
	stepper_set_speed(robot.stepper2, pid + rotation_speed);


//	if(counter < SIZE){
//		angle_log[counter] = robot.mpu->x_angle;
//		pid_log[counter] = pid;
////		time_log[counter] = robot.mpu->lst_update_x_angle;
////		step_counter_log[counter] = robot.stepper1->step_counter;
//		counter ++;
//	}
//	else{
//		robot_stop();
//		for(int i = 0; i < SIZE; i++){
//			uint8_t buffer[BUFFER_SIZE_TX];
//			//uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f, %ld, %ld\r\n", angle_log[i], pid_log[i], step_counter_log[i], time_log[i]);
//			uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f, %d\r\n", angle_log[i], pid_log[i], i);
//
//			HAL_UART_Transmit(uart_interface.huart, buffer, size, HAL_MAX_DELAY);
//		}
//
//	}


	pid_step(robot.pos_pid);
	pid_step(robot.angle_pid);
	pid_step(robot.target_speed_pid);


	if(robot.send_data){
		uint8_t buffer[BUFFER_SIZE_TX];
		//uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f, %.3f, %ld\r\n", robot.mpu->x_angle, mpu_get_acc_x_angle(robot.mpu), robot.mpu->gy, robot.encoder_angle);
		//uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\r\n", robot.mpu->x_angle, robot.target_speed_pid->error, target_angle_filtered, p_pos, robot.pos_pid->i, d_pos, speed, target_speed, p, d);
		uint16_t size = snprintf((char*)buffer, BUFFER_SIZE_TX, "%.3f, %.3f, %.3f\r\n", robot.target_speed_pid->error, target_speed, target_angle_filtered);
		uart_send(&uart_interface, buffer, size, 0);
	}
}




void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){   // DMA transfer complete mpu + encoder
	if(hi2c == robot.mpu->hi2c){
		robot.mpu->ax = (int16_t)((int16_t)I2C_DMA_buffer[0] << 8 | I2C_DMA_buffer[1]) * robot.mpu->acc_scale;
		robot.mpu->az = (int16_t)((int16_t)I2C_DMA_buffer[4] << 8 | I2C_DMA_buffer[5]) * robot.mpu->acc_scale;
		robot.mpu->gy = ((int16_t)((int16_t)I2C_DMA_buffer[10] << 8 | I2C_DMA_buffer[11]) * robot.mpu->gyro_scale) - robot.mpu->gy_bias;
		mpu_calc_x_angle(robot.mpu);
		if(robot.control_on){
			control_loop();
		}
	}
	else if (hi2c == &hi2c2){  // encoder
		//calculate_encoder_angle();
	}

}




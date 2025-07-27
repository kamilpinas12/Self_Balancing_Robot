/*
 * helpers.h
 *
 *  Created on: Nov 14, 2024
 *      Author: kamil
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_



#define TIME_DELTA 0.01 //[s]
#define WHEEL_RADIUS 0.03 //[m]
#define STEP2RAD 0.003925
#define STEP2METERS 0.0001178
#define METERS2STEP 8488.2636
#define RAD2METERS 0.1885
#define METERS2RAD 5.3051




extern volatile uint8_t encoder_data_buffer[2];
extern volatile float encoder_angle;








void calculate_encoder_angle();

void saturation(float min, float max, float* val);






#endif /* INC_HELPERS_H_ */

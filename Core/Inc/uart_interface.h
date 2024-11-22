/*
 * robot_interface.h
 *
 *  Created on: Nov 17, 2024
 *      Author: kamil
 */

#ifndef INC_UART_INTERFACE_H_
#define INC_UART_INTERFACE_H_


#include <stdbool.h>

#include "main.h"


#define BUFFER_SIZE 100
#define FUNCTION_CODE_MAX_LENGTH 16
#define ARG_MAX_LENGTH 12
#define MAX_NUM_ARGS 5



// number of user functions
#define NUM_FUNCTIONS 7



typedef struct{
	void (*function_pointer)(char [MAX_NUM_ARGS][ARG_MAX_LENGTH]);
	char function_code[FUNCTION_CODE_MAX_LENGTH];
	uint8_t num_args;

}user_function_typedef;




typedef struct{
	UART_HandleTypeDef *huart;
	uint8_t receive_buffer[BUFFER_SIZE];
	uint16_t commnad_size;
    volatile bool command_received_flag;

    //user functions
    user_function_typedef* functions_array;
    uint8_t num_functions;

}uart_interface_typedef;




void uart_interface_init(uart_interface_typedef* uart_int, UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usart_rx, user_function_typedef* functions_array, uint8_t num_functions);

void bluetooth_setup();

int execute_uart_command(uart_interface_typedef* uart_int);

void help(uart_interface_typedef* uart_int);

void start_uart_interface(uart_interface_typedef* uart_int);




#endif /* INC_UART_INTERFACE_H_ */

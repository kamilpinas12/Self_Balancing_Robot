/*
 * robot_interface.c
 *
 *  Created on: Nov 17, 2024
 *      Author: kamil
 */

#include<stdio.h>
#include<string.h>
#include<stdlib.h>

#include "stm32f1xx_hal.h"

#include <uart_interface.h>


extern uart_interface_typedef uart_interface;




void start_uart_interface(uart_interface_typedef* uart_int){
	HAL_UARTEx_ReceiveToIdle_DMA(uart_int->huart, uart_int->receive_buffer, BUFFER_SIZE);
}



// UART IDLE interrupt
//if you want to receive data from multiple uart add another if statement for your uart and uart_interface_typedef structure


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == uart_interface.huart->Instance)
    {
    	uart_interface.command_received_flag = 1;
    	uart_interface.commnad_size = Size;

    }
}


void uart_interface_init(uart_interface_typedef* uart_int, UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usart_rx, user_function_typedef* functions_array, uint8_t num_functions){
	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT); //disable half transfer dma interrupt

	uart_int->huart = huart;
	uart_int->commnad_size = 0;
	uart_int->command_received_flag = 0;
	uart_int->functions_array = functions_array;
	uart_int->num_functions = num_functions;

}



void bluetooth_setup(){
	// ustawiania parametrów modułu nie działa

}


/*
 * no command to parse, returns -1
 * can't parse command (error), returns 0
 * successful command read,  returns 1
 * command not found , returns 2
 */

int execute_uart_command(uart_interface_typedef* uart_int){
	if(!uart_int->command_received_flag) return -1;

	uart_int->command_received_flag = 0;

	char function_code[FUNCTION_CODE_MAX_LENGTH];
	char args[MAX_NUM_ARGS][ARG_MAX_LENGTH];


	uint8_t idx = 0;
	// parse function code
	for(int i = 0; i < uart_int->commnad_size || i < FUNCTION_CODE_MAX_LENGTH; i++){
		idx ++;
		char elem = uart_int->receive_buffer[i];
		if(elem == '\0' || elem == '\r' || elem == '\n' || elem == '('){
			function_code[i] = '\0';
			break;
		}
		else function_code[i] = elem;
	}


	//parse arguments
	bool no_args = 1;
	uint8_t arg_len = 0;
	uint8_t arg_counter = 0;
	for(int i = idx; i < uart_int->commnad_size; i++)
	{
		//safety checks
		if(arg_counter >= MAX_NUM_ARGS || arg_len >= ARG_MAX_LENGTH) return 0; //error, to many argumenst/to long argument

		char elem = uart_int->receive_buffer[i];
		if(elem == '\0' || elem == '\r' || elem == '\n') return 0; //error, commands ends with ')'

		else if(elem == ')'){
			args[arg_counter][arg_len] = '\0';
			break;
		}
		else if(elem == ','){
			args[arg_counter][arg_len] = '\0';
			arg_counter++;
			arg_len = 0;
		}
		else if(elem == ' '){
			continue;
		}
		else{
			no_args = 0;
			args[arg_counter][arg_len] = elem;
			arg_len ++;
		}
	}
	if(!no_args) arg_counter++;

	if(strcmp(function_code, "help") == 0){
		help(uart_int);
		return 1;
	}



	//run
	for(int i = 0; i < uart_int->num_functions; i++){
		if(strcmp(function_code, uart_int->functions_array[i].function_code) == 0){
			if(uart_int->functions_array[i].num_args == arg_counter){
				(*uart_int->functions_array[i].function_pointer)(args);
				return 1;
			}
			break;
		}
	}

//	printf("function code:[%s]\n", function_code);
//
//	for(int i = 0; i < arg_counter; i++){
//		printf("arg%d = [%s]\n", i, args[i]);
//	}

	return 2;
}




void help(uart_interface_typedef* uart_int){
	for(int i = 0; i < uart_int->num_functions; i++){
		printf("%s : %d arg\n", uart_int->functions_array[i].function_code, uart_int->functions_array[i].num_args);
	}
}








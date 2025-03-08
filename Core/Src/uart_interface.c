/*
 * robot_interface.c
 *
 *  Created on: Nov 17, 2024
 *      Author: kamil
 */

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<stdbool.h>

#include "stm32f1xx_hal.h"

#include <uart_interface.h>


extern uart_interface_typedef uart_interface;



void uart_interface_init(uart_interface_typedef* uart_int, UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usart_rx,
		user_function_typedef* functions_array, uint8_t num_functions){

	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT); //disable half transfer DMA interrupt

	uart_int->huart = huart;
	uart_int->received_command_size = 0;
	uart_int->command_received_flag = 0;
	uart_int->functions_array = functions_array;
	uart_int->num_functions = num_functions;
	uart_int->uart_tx_ready = 1;
	uart_int->queue_empty = 1;


}


void start_uart_interface(uart_interface_typedef* uart_int){
	HAL_UARTEx_ReceiveToIdle_DMA(uart_int->huart, uart_int->receive_buffer, BUFFER_SIZE_RX);
}


void uart_send(uart_interface_typedef* uart_int, uint8_t* buffer, uint16_t size, bool add_to_queue){
	if(size < BUFFER_SIZE_TX){
		if(uart_int->uart_tx_ready){
			uart_int->uart_tx_ready = 0;
			memcpy(uart_int->transmit_buffer, buffer, size);
			HAL_UART_Transmit_DMA(uart_int->huart, uart_int->transmit_buffer, size);
		}
		else if(add_to_queue && uart_int->queue_empty){
			memcpy(uart_int->queue, buffer, size);
			uart_int->queue_data_size = size;
			uart_int->queue_empty = 0;
		}
	}
}


/*
 * command received interrupt
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == uart_interface.huart->Instance)
    {
    	uart_interface.command_received_flag = 1;
    	uart_interface.received_command_size = Size;

    }
}

/*
 * transnmit completed, if there is element in queue buffer send it, otherwise set uart transmit to ready
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == uart_interface.huart->Instance){
		if(uart_interface.queue_empty == 0){
			memcpy(uart_interface.transmit_buffer, uart_interface.queue, uart_interface.queue_data_size);
			HAL_UART_Transmit_DMA(uart_interface.huart, uart_interface.transmit_buffer, uart_interface.queue_data_size);
			uart_interface.queue_empty = 1;
			uart_interface.uart_tx_ready = 0;
		}
		else{
			uart_interface.uart_tx_ready = 1;
		}
	}
}


int8_t execute_received_command(uart_interface_typedef* uart_int){
	if(!uart_int->command_received_flag) return 0;

	uart_int->command_received_flag = 0;

	char function_code[FUNCTION_CODE_MAX_LENGTH];
	char args[MAX_NUM_ARGS][ARG_MAX_LENGTH];


	// parse function code
	uint8_t idx = 0;
	for(int i = 0; i < uart_int->received_command_size || i < FUNCTION_CODE_MAX_LENGTH; i++){
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
	for(int i = idx; i < uart_int->received_command_size; i++)
	{
		if(arg_counter >= MAX_NUM_ARGS || arg_len >= ARG_MAX_LENGTH) return -1; //error, to many argumenst/to long argument

		char elem = uart_int->receive_buffer[i];
		if(elem == '\0' || elem == '\r' || elem == '\n') return -1; //error, commands ends with ')'

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


	//run received function
	for(int i = 0; i < uart_int->num_functions; i++){
		if(strcmp(function_code, uart_int->functions_array[i].function_code) == 0){
			if(uart_int->functions_array[i].num_args == arg_counter){
				(*uart_int->functions_array[i].function_pointer)(args);
				return 1;
			}
			break;
		}
	}
	return -1;
}



/*
 * send "help" command to stm, stm will transmit avaible commands and number of arguments for each command
 */

void help(uart_interface_typedef* uart_int){
	uint16_t offset = 0;
	uint8_t buffer[BUFFER_SIZE_TX];
	for (int i = 0; i < uart_int->num_functions; i++) {
		int16_t written = snprintf((char*)(buffer + offset), BUFFER_SIZE_TX - offset,
							   "%s - num_args: %d\n",
							   uart_int->functions_array[i].function_code,
							   uart_int->functions_array[i].num_args);

		if (written < 0 || offset >= BUFFER_SIZE_TX){
			break;
		}

		offset += written;
	}
	uart_send(uart_int, buffer, offset, 1);
}



void pos_hold(char args[MAX_NUM_ARGS][ARG_MAX_LENGTH]){



}





/*
 * communication_protocol.c
 *
 *  Created on: Jul 21, 2025
 *      Author: kamil
 */


#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<stdbool.h>

#include "stm32f1xx_hal.h"

#include"communication_interface.h"
#include"robot.h"


extern robot_t robot;


communication_interface_t communication_interface;



reg_typedef registerMap[REGISTER_MAP_SIZE];








static void start_interface(){
	HAL_UARTEx_ReceiveToIdle_DMA(communication_interface.huart, communication_interface.receive_buffer, Rx_BUFFER_SIZE);
}



void communication_interface_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usart_rx){
	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT); //disable half transfer DMA interrupt

	communication_interface.huart = huart;
	communication_interface.received_command_size = 0;
	communication_interface.command_received_flag = 0;

	communication_interface.uart_tx_ready = 1;

	start_interface();
}



void execute_command(){
	if(communication_interface.command_received_flag &&
			communication_interface.received_command_size >= 3 &&
			communication_interface.received_command_size <= 6 &&
			communication_interface.receive_buffer[0] == 0xAA){
				uint8_t address = communication_interface.receive_buffer[1];
				if(address < REGISTER_MAP_SIZE){
					uint8_t size = communication_interface.received_command_size - 2;
					if(registerMap[address].ptr != NULL && registerMap[address].dataSize == size){
						memcpy(registerMap[address].ptr, communication_interface.receive_buffer+2, size);
					}
				}
	}
	communication_interface.command_received_flag = 0;
	start_interface();
	return;

}



void uart_send_buffer(uint16_t size){
	if(communication_interface.uart_tx_ready){
		communication_interface.uart_tx_ready = 0;
		HAL_UART_Transmit_DMA(communication_interface.huart, communication_interface.transmit_buffer, size);
	}
}




/* ###############################################
 * 		INTERRUPT CALLBACKS
 * ###############################################
 */

// received UART command
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == communication_interface.huart->Instance)
    {
    	communication_interface.command_received_flag = 1;
    	communication_interface.received_command_size = Size;
    	start_interface();
    }
}

// UART transmit ready
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == communication_interface.huart->Instance){
		communication_interface.uart_tx_ready = 1;
	}
}








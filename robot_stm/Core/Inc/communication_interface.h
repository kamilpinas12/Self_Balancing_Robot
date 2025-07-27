/*
 * communication_protocol.h
 *
 *  Created on: Jul 21, 2025
 *      Author: kamil
 */

#ifndef INC_COMMUNICATION_INTERFACE_H_
#define INC_COMMUNICATION_INTERFACE_H_




#include<stdio.h>
#include<stdint.h>
#include<stdbool.h>

#include "stm32f1xx_hal.h"



#define Tx_BUFFER_SIZE 256
#define Rx_BUFFER_SIZE 16
#define REGISTER_MAP_SIZE 32




typedef struct{
	void* ptr;
	uint8_t dataSize;
}reg_typedef;



typedef struct{
	UART_HandleTypeDef *huart;
	uint8_t receive_buffer[Rx_BUFFER_SIZE];
	uint8_t transmit_buffer[Tx_BUFFER_SIZE];

	uint16_t received_command_size;
    volatile bool command_received_flag;
    volatile bool uart_tx_ready;

}communication_interface_t;










void communication_interface_init(UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usart_rx);

void execute_command();

void uart_send_buffer(uint16_t size);








#endif /* INC_COMMUNICATION_INTERFACE_H_ */

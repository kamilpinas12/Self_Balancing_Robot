/*
 * robot_interface.h
 *
 *  Created on: Nov 17, 2024
 *      Author: kamil
 */

#ifndef INC_UART_INTERFACE_H_
#define INC_UART_INTERFACE_H_


#include <stdbool.h>



#define BUFFER_SIZE_RX 128
#define BUFFER_SIZE_TX 256


#define FUNCTION_CODE_MAX_LENGTH 16
#define ARG_MAX_LENGTH 12
#define MAX_NUM_ARGS 8

#define TRANSMIT_QUEUE_SIZE 4





typedef struct{
	void (*function_pointer)(char [MAX_NUM_ARGS][ARG_MAX_LENGTH]);
	char function_code[FUNCTION_CODE_MAX_LENGTH];
	uint8_t num_args;

}user_function_typedef;



typedef struct{
	UART_HandleTypeDef *huart;
	uint8_t receive_buffer[BUFFER_SIZE_RX];
	uint8_t transmit_buffer[BUFFER_SIZE_TX];

	uint16_t received_command_size;
    volatile bool command_received_flag;
    volatile bool uart_tx_ready;

    //user functions var
    user_function_typedef* functions_array;
    uint8_t num_functions;

    // transmit queue
    uint8_t queue[BUFFER_SIZE_TX];
    volatile uint8_t queue_data_size;
    volatile bool queue_empty;

}uart_interface_typedef;





void uart_interface_init(uart_interface_typedef* uart_int, UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usart_rx, user_function_typedef* functions_array, uint8_t num_functions);



// start receiving data with uart dma, this function needs to be executed after "execute_received_function"
void start_uart_interface(uart_interface_typedef* uart_int);





/*
 * Send data using UART and DMA.
 * If the message is important, set add_to_queue to 1.
 * If UART is busy, the data will be sent immediately after UART becomes ready.
 *
 * Note: The queue stores only one element. If UART is busy and the queue already contains data, the uart_send request will be ignored
 *
 * The queue mechanism was created because if we continuously send data over UART and send a command to the microcontroller
 * that triggers a response, UART will almost always reject it since data is already being transmitted.
 * Therefore, the response is added to the queue, and as soon as UART is ready, the response will be sent.
 * Adding to the queue is recommended only for data that must be transmitted.
 *
 */
void uart_send(uart_interface_typedef* uart_int, uint8_t* buffer, uint16_t size, bool add_to_queue);




/*
 *	run this function to execute received uart command
 *	return 0 - there is no command to execute
 *	return -1 - error (incorrect command, command not found, ...)
 *	return 1 - succes
 */
int8_t execute_received_command(uart_interface_typedef* uart_int);


/*
 * send "help()" command to stm, stm will transmit avaible commands and number of arguments for each command
 */
void help(uart_interface_typedef* uart_int);







#endif /* INC_UART_INTERFACE_H_ */

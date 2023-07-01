
/*
 * UART.h
 *
 *  Created on: Jun 12, 2023
 *      Author: mique
 */

#ifndef INC_UART_H_
#define INC_UART_H_


#define MAX_BUFF_SIZE 100
#define PA2_PA15 1
#define PA2_PA3 2

#endif /* INC_UART2_CONFIG_H_ */

#include <math.h>
#include "main.h"
#include <stdio.h>
#include <stdint.h>


void UART2_Initialization(uint32_t BAUD, uint8_t portComb, char end_of_msg);
uint8_t UART2_setDataBits(uint8_t bits);
uint8_t UART2_setStopBits(uint8_t bits);
uint8_t UART2_setParity(unsigned char Config);
void UART2_Transmit(char data);
void UART2_TransmitString(char data[MAX_BUFF_SIZE]);
void UART2_Receive(unsigned char *data);
void UART2_ReceiveString(unsigned char data[MAX_BUFF_SIZE]);
void UART2_Receive(unsigned char *data);
void UART2_ReceiveString(unsigned char data[MAX_BUFF_SIZE]);
void UART2_Flush(void);
uint8_t UART2_AvailableBuffer(unsigned char data[MAX_BUFF_SIZE], uint8_t cks);
uint8_t checksum(unsigned char buff[MAX_BUFF_SIZE]);
void float_to_str(float number, char* buffer, uint8_t decimalDigits);


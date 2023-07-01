/*
 * UART.c
 *
 *  Created on: Jun 12, 2023
 *      Author: mique
 */

#include "UART.h"

char end;
unsigned char buffer[MAX_BUFF_SIZE] = ""; //buffer global
int buffer_index = 0;

extern float fCLK;


void UART2_Initialization(uint32_t BAUD, uint8_t portComb, char end_of_msg){
	/*----------------------------------------------------------------------------
	 *                     Inicializa configuración de UART2
	 *                     - portComb = 1: configura PA2 (Tx) y PA15 (Rx)
	 *                     - portComb = 2: configura PA2 (Tx) y PA3 (Rx)
	 *NOTA: la idea es que se invoquen con las constantes definidas PA2_PA15 y PA2_PA3
	 ----------------------------------------------------------------------------*/
	USART2->CR1 &= ~USART_CR1_UE; //USART2 disable just in case
	end = end_of_msg;



	//--------------------1. Habilitacion de los clock----------------------------
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // habilito TIM2 clock. [Page 220]
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // habilito GPIOA clock. [page 218]

	//---------------------2. Configuracion de los pines--------------------------
	// configuro PA2 como AFM
	GPIOA->MODER &= ~GPIO_MODER_MODE2;
	GPIOA->MODER |= GPIO_MODER_MODE2_1;

	// configuro la velocidad
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2_Msk;

	// configuro el AF del pin PA2 en AF7
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2 ;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL2_3;
	switch(portComb){
	case 1:
		// configuro PA15 como AFM
		GPIOA->MODER &= ~GPIO_MODER_MODE15;
		GPIOA->MODER |= GPIO_MODER_MODE15_1;

		// configuro la velocidad
		GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED15_Msk;

		// configuro el AF del pin PA15 en AF3
		GPIOA->AFR[1] |= GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_1;
		GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL15_2 | GPIO_AFRH_AFSEL15_3);
		break;
	case 2:
		// configuro PA3 como AFM
		GPIOA->MODER &= ~GPIO_MODER_MODE3_Msk;
		GPIOA->MODER |= GPIO_MODER_MODE3_1;

		// configuro la velocidad
		GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED3_Msk;

		// Configure the AF7 (Alternate Function 7) for PA3
		GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL3_Msk;
		GPIOA->AFR[0] |= (7U << GPIO_AFRL_AFSEL3_Pos);
		break;
	}


	//------------------------3. Configuracion de la UART--------------------------
	// por default se configura en 8N1
	UART2_setDataBits(8); // 8
	UART2_setParity(0); // N
	UART2_setStopBits(0); // 1

	// configuro el baud-rate
	USART2->CR1 &= ~USART_CR1_OVER8; // oversampling seleccionado en 16 [Page 1238]
	// ecuacion ahora es Tx/Rx baud = fCLK/USARTDIV donde USARTDIV = BRR
	USART2->BRR = floor(fCLK/BAUD);
	// NOTA: si oversampling = 8 entonces USARTDIV = 2*BRR

	// dando habilitacion para USART, transmisor y receptor [Page 1241]
	USART2->CR1 |= USART_CR1_UE; //USART2 enable
	USART2->CR1 |= USART_CR1_TE; // Transmitter enable
	USART2->CR1 |= USART_CR1_RE; // Receiver enable
/*	USART2->CR1 |= USART_CR1_RXNEIE; // Habilito interrupcion

	NVIC_SetPriority(USART2_IRQn, 0);
	NVIC_EnableIRQ(USART2_IRQn);*/
}

/*void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        USART2->ISR &= ~USART_ISR_RXNE;

		char receivedChar = USART2->RDR;

		if (buffer_index < MAX_BUFF_SIZE - 1) {
			buffer[buffer_index] = receivedChar;
			buffer_index++;
		}
    }
}*/

uint8_t UART2_setDataBits(uint8_t bits){
	/*----------------------------------------------------------------------------
	 *                     Selecciona los bits de datos
	 ----------------------------------------------------------------------------*/
	if (bits == 7 || bits == 8 || bits == 9){
		switch(bits){
		case 7:
			USART2->CR1 |= USART_CR1_M1; //M1 = 1
			USART2->CR1 &= ~USART_CR1_M0; //M1 = 0
			break;
		case 8:
			USART2->CR1 &= ~USART_CR1_M1; //M1 = 0
			USART2->CR1 &= ~USART_CR1_M0; //M1 = 0
			break;
		case 9:
			USART2->CR1 &= ~USART_CR1_M1; //M1 = 0
			USART2->CR1 |= USART_CR1_M0; //M1 = 1
			break;
		}
		return 1; // pudo aplicar configurarion
	} else {
		return -1; // no pudo aplicar configurarion
	}
}

uint8_t UART2_setStopBits(uint8_t bits){
	/*----------------------------------------------------------------------------
	 *                     Selecciona los bits de stop
	 *                     0 -> 1 stop bit
	 *                     1 -> 0.5 stop bit
	 *                     2 -> 2 stop bit
	 *                     3 -> 1.5 stop bit
	 ----------------------------------------------------------------------------*/
	if (bits == 0 || bits == 1 || bits == 2 || bits == 3){
			USART2->CR2 = (bits << USART_CR2_STOP_Pos);
			return 1;
	} else {
		return -1;
	}
}

uint8_t UART2_setParity(unsigned char Config){
	/*----------------------------------------------------------------------------
	 *                          Selecciona la paridad
	 *                          - Config -> 0..2
	 *                          	- Config = 00b o 0d: sin paridad
	 *                          	- Config = 01b o 1d: con paridad par
	 *                          	- Config = 10b o 2d: con paridad impar
	 ----------------------------------------------------------------------------*/
	if (Config == 0 || Config == 1 || Config == 2){
		switch(Config){
		case 0:
			USART2->CR1 &= ~USART_CR1_PCE;
			break;
		case 1:
			USART2->CR1 |= USART_CR1_PCE;
			USART2->CR1 &= ~USART_CR1_PS;
			break;
		case 2:
			USART2->CR1 |= USART_CR1_PCE;
			USART2->CR1 |= USART_CR1_PS;
			break;

		}
		return 1;
	} else {
		return -1;
	}
}

void UART2_Transmit(char data){
	/*----------------------------------------------------------------------------
	 * 							Transmite un solo caracter
	 ----------------------------------------------------------------------------*/
	USART2->TDR = data;
	while(!(USART2->ISR & USART_ISR_TXE));
}

void UART2_TransmitString(char data[MAX_BUFF_SIZE]){
	/*----------------------------------------------------------------------------
	 * 							Transmite un string completo
	 ----------------------------------------------------------------------------*/
	    for (int i = 0; i < MAX_BUFF_SIZE; i++) {
	    	if(data[i] == end){
	    		UART2_Transmit(data[i]);
	    		break;
	    	}
 	    	UART2_Transmit(data[i]);
	    }
}

void UART2_Receive(unsigned char *data){
	/*----------------------------------------------------------------------------
	 * 							Recibe sin interrupciones
	 ----------------------------------------------------------------------------*/
		// espera hasta que el dato este disponible
		while (!(USART2->ISR & USART_ISR_RXNE));

		// lee el dato
		*data = USART2->RDR;
}

void UART2_ReceiveString(unsigned char data[MAX_BUFF_SIZE]) {
	/*----------------------------------------------------------------------------
	 * 						Recibe strings sin interrupciones
	 ----------------------------------------------------------------------------*/
    int i = 0;
    while (i < MAX_BUFF_SIZE - 1) {
        while (!(USART2->ISR & USART_ISR_RXNE)); // espera a que haya recibido
        data[i] = USART2->RDR; // lee el dato recibido

        if (data[i] == '\r') {
            data[i] = '\r'; //
            break;
        }
        i++;
    }
}

uint8_t UART2_AvailableBuffer(unsigned char data[MAX_BUFF_SIZE], uint8_t cks){
	/*----------------------------------------------------------------------------
	 * 						Verifica si el buffer esta disponible
	 * 						cks = 1: realiza checksum para saber si esta disponible
	 * 						cks = 0: no realiza checksum
	 *NOTA: se asume disponible si el checksum es correcto
	 ----------------------------------------------------------------------------*/
	for (unsigned int i = 0; i < MAX_BUFF_SIZE; i++) { // recorre el arreglo
		if (buffer[i] == end){ // verifica si encuentra el final del mensaje '\r'
	        if (cks == 0 || (cks == 1  && checksum(buffer) == 0)){
		        for (unsigned int i = 0; i < buffer_index; i++) {
		            data[i] = buffer[i]; // 'copia en el buffer si lo ha encontrado y esta disponible'
		        }
	        	return buffer_index;
	        } else if (cks == 1 && checksum(buffer) != 0) {
	        	return 0; // retorna '0' si no esta disponible
	        }
		}
	}
	return 0;
}

void UART2_Flush() {
	/*----------------------------------------------------------------------------
	 * 						Limpia el buffer completo
	 ----------------------------------------------------------------------------*/
    for (unsigned int i = 0; i < MAX_BUFF_SIZE; i++) {
        buffer[i] = '\0';
    }
    buffer_index = 0;
}

uint8_t checksum(unsigned char buff[MAX_BUFF_SIZE]){
	/*----------------------------------------------------------------------------
	 * 						Realiza el checksum del buffer
	 *NOTA: asume que el checksum esta al final del buffer.
	 ----------------------------------------------------------------------------*/
	char cs_received = 0;
	uint8_t cs = 0;

	cs_received = buff[buffer_index-2];

	for (int i = 0; i < buffer_index-2; i++) {
	  cs = cs + buff[i]; // Se realiza el checksum
	}
	return cs_received + cs;
}


void float_to_str(float number, char* buffer, uint8_t decimalDigits){
    int32_t integerPart = (int32_t)number;
    float decimalPart = number - integerPart;
    long int decimal = 0;
    char buffer_decimal[50] = "";
    char buffer_zeros[50] = "";
    int counter = 0;

    // Convert integer part to string
    sprintf(buffer, "%ld", integerPart);

    // Add decimal point if decimalDigits is greater than 0
    if (decimalDigits > 0) {
        strcat(buffer, ".");

        // Convert decimal part to string with specified decimal digits
        for (uint8_t i = 0; i < decimalDigits; i++) {
            decimalPart *= 10;
            int32_t decimalDigit = (int32_t)decimalPart;

            if (decimalDigit == 0) {
            	counter++;
            }

            decimalPart -= decimalDigit;
            decimal = decimal*10+decimalDigit;
        }
        for (int i = 0; i<counter; i++){
        	strcat(&buffer_zeros, "0");
        }
        sprintf(&buffer_decimal,"%ld",decimal);
        strcat(buffer_zeros,buffer_decimal);
        strcat(buffer,buffer_zeros);
    }
    strcat(buffer,"\r");
}



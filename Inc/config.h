/*
 * config.h
 *
 *  Created on: Jun 7, 2023
 *      Author: mique
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "main.h"
#include <math.h> // Para trabajar con float

void GPIO_Config(void);
void DAC_Config(void);
void set_voltage_DAC(uint16_t voltageOUT);
void ADC_Config(void);
float read_ADC();

void TIM6_Config(int arr, int psc);
void enable_TIM6(void);
void disable_TIM6(void);
float get_seconds_TIM6(void);

void on_Pin_A1(void);
void off_Pin_A1(void);
void delay_us(uint32_t wait);
void delay_ms(uint32_t wait);

#endif /* INC_CONFIG_H_ */





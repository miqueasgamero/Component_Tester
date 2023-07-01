/*
 * measurements.c
 *
 *  Created on: Jun 7, 2023
 *      Author: mique
 */

#include "measurements.h"
#include "config.h"

extern float VREF;

static float known_R = 1000;
static float uknown_R = 0;

float get_resistance(void){
 //ApagarPinA1();
 //while(read_ADC() != 0); //Esto para que no se pueda leer sin resistencia, porque el ADC lee cualquier cosa en vacío
 //Las lineas de arriba no las puse porque habria que cortar el While en algún momento
 on_Pin_A1();
 delay_ms(1000); //Espero un segundo para leer el ADC, para que la tensión en el pin PA1 sea 3.3V
 float T2 = read_ADC();
 T2 = (3.3*T2)/4095.0;
 uknown_R = (T2*known_R)/(3.3 - T2);
 off_Pin_A1();
 return uknown_R;
}

float get_capacitance(void){
	float tau = 0;

	// Espero a que se descargue el capacitor
	while(read_ADC()>(3.3/4095.0)*10){
		off_Pin_A1();
	}
	on_Pin_A1(); // Energizo el circuito

	// Espero a que el capacitor se carga al 63%
	enable_TIM6();
	while(read_ADC()<0.63*VREF){
		// Espero
	}
	// Leo el tiempo de carga al 63%
	tau = get_seconds_TIM6();

	disable_TIM6();

	return tau/known_R; // C = tau/R
}

void change_known_resistance(float Res){
	known_R = Res;
}

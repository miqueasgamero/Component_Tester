/*
 * config.c
 *
 *  Created on: Jun 7, 2023
 *      Author: mique
 */

#include "config.h"

extern float fCLK;
extern float VREF;

// Variables globales para el manejo del timer
extern float T_OF; //T_OF es el tiempo que tarda TIM6 en hacer un overflow: (ARR*PSC)/fCLK
extern float T_ARR; // T_ARR cuantos segundos tarda una cuenta del TIM6: T_OF/ARR
extern float counter_OF; // counter_OF cuenta cuantos overflow hay en el TIM6

void GPIO_Config(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // habilito GPIOA clock. [page 218]

	// Configuro PA0 como analog-mode
	GPIOA->MODER |= GPIO_MODER_MODE0_Msk;

	// Configure PA1 as output mode without pull-up/pull-down
	GPIOA->MODER &= ~(GPIO_MODER_MODE1_Msk);
	GPIOA->MODER |= GPIO_MODER_MODE1_0;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD1_Msk); // No pull-up/pull-down
}

void DAC_Config(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN; //Habilito DAC clock

  // Configure the DAC channel
  DAC1->CR &= ~DAC_CR_TEN1;  // Deshabilito external trigger
  //DAC1->CR &= ~DAC_CR_TSEL1; // Este registro es importante si trabajas con external trigger.

  // Set the DAC data
  DAC1->DHR12R1 = 0x000;  // En alineacion a la derecha 12 bit coloca una tension de 0V en la salida del DAC

  // Enable the DAC channel
  DAC1->CR |= DAC_CR_EN1;

}

void set_voltage_DAC(uint16_t voltageOUT){
    // Modify the output voltage of the DAC
    DAC1->DHR12R1 = voltageOUT;  // Set the output voltage to half scale

    // Trigger the DAC conversion
    DAC1->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

void ADC_Config(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN; // habilito ADC clock [Page 218, Sec 6.4.16]

    // Setea el clock del sistema como el clock del ADC [Page 234]
    RCC->CCIPR |= RCC_CCIPR_ADCSEL_0;
    RCC->CCIPR |= RCC_CCIPR_ADCSEL_1;

    // Common Control Register
    ADC1_COMMON->CCR &= ~ADC_CCR_PRESC_Msk;
    ADC1_COMMON->CCR |= (0B0000 << ADC_CCR_PRESC_Pos); // Prescaler = 0000, i.e. sin division

    // Control Register
    ADC1->CR &= ~ADC_CR_DEEPPWD; // ADC not in Deep-power down
    ADC1->CR |= ADC_CR_ADVREGEN; // ADC volt regulator enable
    delay_us(20); // Deberia esperar 20useg, espero 1ms.

    // Configuration Register
    ADC1->CFGR &= ~ADC_CFGR_RES; // Resolucion de 12-bit [Page 461]

    // Sample Time Register
    ADC1->SMPR1 |= (0B010 << ADC_SMPR1_SMP5_Pos); // El canal del PA0 es el 5 en 12.5 ADC clock cycles

    // Regular Sequence Register
    ADC1->SQR1 &= ~ADC_SQR1_L; // Set 1 conversion
    ADC1->SQR1 |= (0B0101 << ADC_SQR1_SQ1_Pos); // Setea conversion sequence para canal 5 [Page 468]

    // Control Register
    ADC1->CR |= ADC_CR_ADCAL; // Calibracion del ADC. Necesario para mejorar las lecturas. [Page 454]
    while((ADC1->CR & ADC_CR_ADCAL)); // Espero la terminacion de la calibracion.  [Page 454]
}

float read_ADC(void){
	// Instrucciones en [Page 386]
	ADC1->ISR |= ADC_ISR_ADRDY; // Limpio ADRDY seteandolo en 1
	ADC1->CR |= ADC_CR_ADEN; // Habilito ADC una vez terminada la configuracion y calibracion
	delay_ms(1);

	ADC1->CR |= ADC_CR_ADSTART;

    while(~(ADC1->ISR & ADC_ISR_EOC) == 0); // Espero al fin de la conversion

    // Clearing EOS y ADRDY
    ADC1->ISR |= ADC_ISR_EOS;
    ADC1->ISR |= ADC_ISR_ADRDY;

    return ADC1->DR*(VREF/4095.0);
}

void TIM6_DAC_IRQHandler(void){
	if(TIM6->SR & TIM_SR_UIF){
		counter_OF++;
	}
	TIM6->SR &= ~TIM_SR_UIF;
}

void TIM6_Config(int arr, int psc){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
	NVIC->ISER[1] |= (1 << 22);
	TIM6->CR1 = TIM_CR1_ARPE;
	TIM6->ARR = arr-1; // Auto-Reload Register
	TIM6->PSC = psc-1; // Prescaler

	T_OF = (arr+psc)/fCLK;
	T_ARR = T_OF/arr;
}

float get_seconds_TIM6(void){
	uint16_t stop_time = 0;
	float seconds = 0;

	// Leo el valor del contador del clock
	stop_time = TIM6->CNT;

	seconds = T_OF*counter_OF+stop_time*T_ARR;

	// Una vez parado el "cronometro" implementado con el TIM6, se reinicia el contador de interrupciones
	counter_OF = 0;

	return seconds;
}

void enable_TIM6(void){
	// Le doy enable a TIM6
    TIM6->CR1 |= TIM_CR1_CEN;

    // Habilito interrupciones en TIM6
	TIM6->EGR = TIM_EGR_UG;
	TIM6->DIER = TIM_DIER_UIE;

	// Reseteo el contador del TIM6
	// NOTA: EGR_UF ya limpia el contador, esto es redundancia por seguridad
	TIM6->CNT = 0;
}

void disable_TIM6(void){
	// Deshabilito el TIM6
    TIM6->CR1 &= ~TIM_CR1_CEN;

    // Deshabilito la interrupcion de TIM6
	TIM6->DIER &= ~TIM_DIER_UIE;
}

void on_Pin_A1(void){
	GPIOA->ODR |= GPIO_ODR_OD1;
}

void off_Pin_A1(void){
	GPIOA->ODR &= ~GPIO_ODR_OD1;
}

void delay_us(uint32_t wait){
	SysTick->LOAD = wait*10;
	SysTick->VAL = 0;
	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}

void delay_ms(uint32_t wait){
	for (uint32_t i = wait; i > 0; i--)
		delay_us(1000);
}




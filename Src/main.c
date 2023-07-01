/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "main.h"

#include "stm32l4xx_hal.h"
#include "config.h"
#include "measurements.h"
#include "UART.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ConfigurarClock(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile const float fCLK = 80000000.0;
volatile const float VREF = 3.3;

volatile float counter_OF = 0; // Controla cuantas veces ha ingresado a la TIM6_DAC_IRQHandler
volatile float T_OF = 0; //T_OF es el tiempo que tarda TIM6 en hacer un overflow: (ARR*PSC)/fCLK
volatile float T_ARR = 0; // T_ARR cuantos segundos tarda una cuenta del TIM6: T_OF/ARR

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  float C = 0.0;
  char buffer[100] = "";
  char end_of_msg = '\r';

  ConfigurarClock();
  UART2_Initialization(9600, PA2_PA15, end_of_msg);
  GPIO_Config();
  ADC_Config();
  TIM6_Config(65535,10);

  C = get_capacitance();
  float_to_str(C, buffer, 10);

  while (1) {
	  UART2_TransmitString(buffer);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void ConfigurarClock(void) {
  //Configuraciones del clock interno
  // Habilito el oscilador MSI
  RCC->CR |= RCC_CR_MSION;
  while (!(RCC->CR & RCC_CR_MSIRDY));
  // Lo configuro a 4 MHz
  RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE) | (RCC_CR_MSIRANGE_6);

  // Configuro y habilito el PLL

  // Limpio los bits de SRC, PLLM, PLLN y PLLR
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLR);
  // Selecciono el clock MSI como entrada del PLL
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_MSI;
  // Configuro PLLM para que divida por 1:
  RCC->PLLCFGR |= (0b000 << RCC_PLLCFGR_PLLM_Pos);
  // Configuro PLLN para que multiplique por 40
  RCC->PLLCFGR |= (40 << RCC_PLLCFGR_PLLN_Pos);
  // VCO Clock freq = MSI*PLLN/PLLM =4MHz * 40 /2
  // Activo clock del PLL, PLL Clk Freq = VCO_Clk_freq / PLLR
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
  RCC->PLLCFGR |= (0b00 << RCC_PLLCFGR_PLLREN_Pos); // Lo dejo en el minimo (2)
  RCC->CR |= RCC_CR_PLLON;

  // Espero a que el PLL esté listo
  while (!(RCC->CR & RCC_CR_PLLRDY));

  // Seteo flash latency para 80 MHz
  FLASH->ACR |= FLASH_ACR_LATENCY_4WS;
  FLASH->ACR |= FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
  // Selecciono PLL como system clock
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;

  // Espera hasta que el system clock se cambie al PLL
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

  // Habilito SysTick timer para Delays
  SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK_DIV8; // SysTick aumenta cada 1/10MHz
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

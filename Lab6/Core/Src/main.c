/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

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


const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void initLEDs(void);
/* USER CODE BEGIN PFP */
void initPA4(void) {
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_Msk);

	GPIOA->MODER |= (GPIO_MODER_MODER4_Msk);
	
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR4_Msk);
	
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4_Msk);
}


uint8_t index = 0;

void waveDAC(void) {
	DAC1->DHR8R1 = sine_table[index];
	
	index++;
	if (index == 31) {
		index = 0;
	}
	
	HAL_Delay(1);
}


void initDAC(void) {
	DAC1->CR |= (DAC_CR_TSEL1_Msk);
	
	DAC1->CR |= (DAC_CR_EN1_Msk);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  HAL_Init();
  SystemClock_Config();
	
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->AHBENR |= (RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN) ;
	
	initLEDs();
	
	GPIOA->MODER |= (1<<2) | (1<<3);
	GPIOA->PUPDR &= ~((1<<2)|(1<<3));
	
	ADC1->CFGR1 |= (1<<4)|(1<<13);	
	//ADC1->CR |= ADC_CR_ADEN;
	
	ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; 
	ADC1->CR |= ADC_CR_ADCAL; 
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) 
	{
	}
	
	ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
	ADC1->CR |= ADC_CR_ADEN;
	ADC1->CR |= ADC_CR_ADSTART;
	
	
	initPA4();
	initDAC();
	
  uint16_t value = 0;
  while (1)
  {
		
		waveDAC();
		value = ADC1->DR;

		if(value > 204){
			GPIOC->ODR |= (1<<6);
			GPIOC->ODR &= ~((1<<9)|(1<<8)|(1<<7));
		}else if(value > 153){
			GPIOC->ODR |= (1<<8);
			GPIOC->ODR &= ~((1<<9)|(1<<6)|(1<<7));
		}else if(value > 102){
			GPIOC->ODR |= (1<<7);
			GPIOC->ODR &= ~((1<<9)|(1<<6)|(1<<8));
		}else if(value > 51){
			GPIOC->ODR |= (1<<9);
			GPIOC->ODR &= ~((1<<8)|(1<<6)|(1<<7));
		}else{
			GPIOC->ODR &= ~((1<<8)|(1<<6)|(1<<7)|(1<<9));
		}
		
		ADC1->ISR |= (1<<2);
  }
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}



void initLEDs(void) {	
	
	GPIOC->MODER |= ((1 << 12)|(1 << 14) | (1 << 16) | (1 << 18));
	GPIOC->MODER &= ~((1 << 13)|(1 << 15) | (1 << 17) | (1 << 19));
	
	GPIOC->OTYPER &= ~((1 << 6)|(1 << 7) | (1 << 8) | (1 << 9));
	
	GPIOC->OSPEEDR &= ~((1 << 12)|(1 << 14)|(1 << 16)|(1 << 18 ));
	
	GPIOC->PUPDR &= ~((1 << 13)|(1 << 15)|(1 << 12)|(1 << 14)|(1 << 16) | (1 << 18)|(1 << 17) | (1 << 19));
	
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));

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

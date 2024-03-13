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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

volatile uint8_t newdata = 0; 
volatile uint8_t led = NULL , operation = NULL;

void UART3_SendChar(char c){
	USART3->TDR = c;  
	//while (!(USART3->ISR & USART_ISR_TC));
}

void UART3_SendStr(char str[]){
    uint8_t send = 0;
		while(str[send] != '\0'){
			if ((USART3->ISR & USART_ISR_TC) == USART_ISR_TC){
				USART3->TDR = str[send++];
			}
		}
	USART3->ICR |= USART_ICR_TCCF;
}

void USART3_4_IRQHandler(void){
	uint8_t rx_val = (uint8_t)(USART3->RDR); /* Receive data, clear flag */
		if(led == NULL && rx_val >=65 && rx_val <= 122){
			led = rx_val;
			UART3_SendStr("\nCMD:: ");
		}else if(rx_val >= 48 && rx_val <= 50){
		  operation	= rx_val;
			newdata = 1;
		}	 
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

	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	USART3->BRR |= 69;
	USART3->CR1 |= ( USART_CR1_TE | USART_CR1_UE);
	USART3->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);

  GPIOC->MODER |= ((1<<9)|(1<<11));
	GPIOC->MODER |= ((1 << 12)|(1 << 14) | (1 << 16) | (1 << 18));
	GPIOC->MODER &= ~((1 << 13)|(1 << 15) | (1 << 17) | (1 << 19));
	
	GPIOC->OTYPER &= ~((1 << 6)|(1 << 7) | (1 << 8) | (1 << 9));
	
	GPIOC->OSPEEDR &= ~((1 << 12)|(1 << 14)|(1 << 16)|(1 << 18 ));
	
	GPIOC->PUPDR &= ~((1 << 13)|(1 << 15)|(1 << 12)|(1 << 14)|(1 << 16) | (1 << 18)|(1 << 17) | (1 << 19));

	
	GPIOC->AFR[0] |= ((1<<16)|(1<<20));
	
	GPIOC->ODR |= ((1<<6)|(1<<7)|(1<<8)|(1<<9));
	
	uint32_t led_pin = 0;
	UART3_SendStr("\nCMD:: ");
  while (1)
  {		
		//USART3->TDR = (1<<6);
		if(newdata == 1){
				USART3->TDR = operation;
				USART3->TDR = led;
				switch(led){
				case 82:
					if(operation == 48){
						GPIOC->ODR &= ~(1<<6);
					}else if(operation == 49){
						GPIOC->ODR |= (1<<6);
					}else if(operation == 50){
						GPIOC->ODR ^= (1<<6);
					}
					break;
				case 66:
					if(operation == 48){
						GPIOC->ODR &= ~(1<<7);
					}else if(operation == 49){
						GPIOC->ODR |= (1<<7);
					}else if(operation == 50){
						GPIOC->ODR ^= (1<<7);
					}
					break;
				case 71:
					if(operation == 48){
						GPIOC->ODR &= ~(1<<9);
					}else if(operation == 49){
						GPIOC->ODR |= (1<<9);
					}else if(operation == 50){
						GPIOC->ODR ^= (1<<9);
					}
					break;
				case 79:
					if(operation == 48){
						GPIOC->ODR &= ~(1<<8);
					}else if(operation == 49){
						GPIOC->ODR |= (1<<8);
					}else if(operation == 50){
						GPIOC->ODR ^= (1<<8);
					}
					break;
				default:
					if(led >= 65 && led <= 122 ){
						UART3_SendStr("Invalid Input\n");
					}
					
					break;
				}
			
				UART3_SendStr("\nCMD:: ");
				newdata = NULL;
				led = NULL;
				operation = NULL;
		}				
		
  }

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
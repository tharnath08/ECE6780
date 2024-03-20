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
#include "stm32f072xb.h"


void transmitToI2C2(uint8_t slvAddr, uint8_t noOfBits, uint8_t txdrData);
uint8_t receiveFromI2C2(uint8_t slvAddr, uint8_t noOfBits);
void initLEDs(void);
void initI2C2(void);
void part1(void);
void initGyro(void);
void senseGyro(void);
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
		
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  RCC->AHBENR |= (RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOBEN);
	
	// initialize LEDs
	initLEDs();
	// initialize I2C2
	initI2C2();	
	
	// Part 1
	//part1();
	// Part 2 initialize
	initGyro();
	
  while (1)
  {
		// Part 2 Gyro
   senseGyro();
  }
}


uint8_t receiveFromI2C2(uint8_t slvAddr, uint8_t noOfBits){
		
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (slvAddr << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (noOfBits  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 |= (I2C_CR2_RD_WRN_Msk);         			// Set the RD_WRN to read operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit
	
	// Wait until RXNE or NACKF flags are set
	while(1) {
		// Continue if RXNE flag is set
		if ((I2C2->ISR & I2C_ISR_RXNE)) {
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			continue;
		}
	}
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}		
	}
	  
	return I2C2->RXDR;;
}


void transmitToI2C2(uint8_t slvAddr, uint8_t noOfBits, uint8_t txdrData){
	// Clear the NBYTES and SADD bit fields
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	I2C2->CR2 |= (slvAddr << I2C_CR2_SADD_Pos);   	// Set the L3GD20 slave address = slvAddr
	I2C2->CR2 |= (noOfBits  << I2C_CR2_NBYTES_Pos); // Set the number of bytes to transmit = noOfBits
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);        			// Set the RD_WRN to write operation
	I2C2->CR2 |= (I2C_CR2_START_Msk);          			// Set START bit

	// Wait until TXIS or NACKF flags are set
	while(1) {
		// Continue if TXIS flag is set
		if ((I2C2->ISR & I2C_ISR_TXIS)) {
			I2C2->TXDR = txdrData; // Set I2C2->TXDR = txdrData
			break;
		}
		
		// Light ORANGE LED if NACKF flag is set (slave didn't respond)
		if ((I2C2->ISR & I2C_ISR_NACKF)) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			continue;
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}
	}
	
}


/* Initialize all configured peripherals */
void initI2C2(void) {
	
	// Set PB11 - AF1
	GPIOB->MODER |= (GPIO_MODER_MODER11_1);          
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_11);            
	GPIOB->AFR[1] |= (0x1 << GPIO_AFRH_AFSEL11_Pos); 
	
	// Set PB13 - AF5
	GPIOB->MODER |= (GPIO_MODER_MODER13_1);          
	GPIOB->OTYPER |= (GPIO_OTYPER_OT_13);            
	GPIOB->AFR[1] |= (0x5 << GPIO_AFRH_AFSEL13_Pos); 
	
	// Set PB14 - initialize high
	GPIOB->MODER |= (GPIO_MODER_MODER14_0);              
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);  
	GPIOB->ODR |= (1<<14);	

	// Set PC0 - initialize high
	GPIOC->MODER |= (GPIO_MODER_MODER0_0);              
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_0);               
	GPIOC->ODR |= (1<<0);

	
	I2C2->TIMINGR |= (0x1  << I2C_TIMINGR_PRESC_Pos);  
	I2C2->TIMINGR |= (0x13 << I2C_TIMINGR_SCLL_Pos);   
	I2C2->TIMINGR |= (0x0F << I2C_TIMINGR_SCLH_Pos);   
	I2C2->TIMINGR |= (0x2  << I2C_TIMINGR_SDADEL_Pos); 
	I2C2->TIMINGR |= (0x4  << I2C_TIMINGR_SCLDEL_Pos); 
	
	
	I2C2->CR1 |= I2C_CR1_PE; 
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
}



/* Initialize the LEDs*/
void initLEDs(void) {	
	
	GPIOC->MODER |= ((1 << 12)|(1 << 14) | (1 << 16) | (1 << 18));
	GPIOC->MODER &= ~((1 << 13)|(1 << 15) | (1 << 17) | (1 << 19));
	
	GPIOC->OTYPER &= ~((1 << 6)|(1 << 7) | (1 << 8) | (1 << 9));
	
	GPIOC->OSPEEDR &= ~((1 << 12)|(1 << 14)|(1 << 16)|(1 << 18 ));
	
	GPIOC->PUPDR &= ~((1 << 13)|(1 << 15)|(1 << 12)|(1 << 14)|(1 << 16) | (1 << 18)|(1 << 17) | (1 << 19));
	
	GPIOC->ODR &= ~((1<<6)|(1<<7)|(1<<8)|(1<<9));

}



/* Part 1 */
void part1(void) {
	
	transmitToI2C2(0xD2, 0x1, 0x0F);
	uint8_t receivedValue = receiveFromI2C2(0xD2, 0x1);
	
	if (receivedValue == 0xD3) {
		HAL_Delay(500);
		GPIOC->ODR ^= (1<<9);
		HAL_Delay(500);
		GPIOC->ODR ^= (1<<9);
		HAL_Delay(500);
		GPIOC->ODR ^= (1<<9);
	}
	
	
	I2C2->CR2 |= (I2C_CR2_STOP);
} 



/* Part 2 Initialize */
void initGyro(void) {
	//transmitToI2C2(0xD2, 0x2, 0x20); //address of the "CTRL_REG1" = 0x20
	//transmitToI2C2(0xD2, 0x2, 0x0B); //address of the "normal or sleep mode" = 0x0B
	
	
	I2C2->CR2 |= (0xD2 << I2C_CR2_SADD_Pos);   
	I2C2->CR2 |= (0x2  << I2C_CR2_NBYTES_Pos); 
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN_Msk);
	I2C2->CR2 |= (I2C_CR2_START_Msk);
	
	// Wait until TXIS or NACKF flags are set (1)
	while(1) {
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = 0x20;
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	
	// Wait again until TXIS or NACKF flags are set (2)
	while(1) {
		if (I2C2->ISR & I2C_ISR_TXIS) {
			I2C2->TXDR = 0x0B;
			break;
		}
		
		if (I2C2->ISR & I2C_ISR_NACKF) {
		}
	}
	
	// Wait for TC flag is set
	while(1) {
		if (I2C2->ISR & I2C_ISR_TC) {
			break;
		}	
	}
	
	
}




uint8_t x_byte1;
uint8_t x_byte2;
int16_t x;
int16_t x_dir = 0;

uint8_t y_byte1;
uint8_t y_byte2;
int16_t y;
int16_t y_dir = 0;


/* Part 2 Gyro */
void senseGyro(void) {
	transmitToI2C2(0xD2, 0x1, 0x28); // OUT_X_L = 0x28
	x_byte1 = receiveFromI2C2(0xD2, 0x1);
	
	transmitToI2C2(0xD2, 0x1, 0x29); // OUT_X_H = 0x29
	x_byte2 = receiveFromI2C2(0xD2, 0x1);
	
	transmitToI2C2(0xD2, 0x1, 0x2A); // OUT_Y_L = 0x2A
	y_byte1 = receiveFromI2C2(0xD2, 0x1);
	
	transmitToI2C2(0xD2, 0x1, 0x2B); // OUT_Y_H = 0x2B
	y_byte2 = receiveFromI2C2(0xD2, 0x1); 
	
		
	x = (x_byte2 << 8) | (x_byte1 << 0);
	x_dir += x;
	
	y = (y_byte2 << 8) | (y_byte1 << 0);
	y_dir += y;
	
	/***********************************************************************/
	
	if (x_dir < -20) {
		GPIOC->ODR |= (1<<8);
		GPIOC->ODR &= ~(1<<9);
	} else if (x_dir > 20){
		GPIOC->ODR |= (1<<9);
		GPIOC->ODR &= ~(1<<8);
	} else{
		GPIOC->ODR |= (1<<8);
		GPIOC->ODR |= (1<<9);
	}
	
	if (y_dir < -20) {
		GPIOC->ODR |= (1<<6);
		GPIOC->ODR &= ~(1<<7);
	} else if((y_dir > 20)){
		GPIOC->ODR |= (1<<7);
		GPIOC->ODR &= ~(1<<6);
	}else{
		GPIOC->ODR |= (1<<6);
		GPIOC->ODR |= (1<<7);
	}
	
	HAL_Delay(100);
}




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

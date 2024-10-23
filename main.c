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
#define ARR_VALUE 0xFFFFFFFF
#define CCR1_VALUE 1000
#define AF5 5
#define MAX_VOLTAGE 3300
#define MAX_TWELVE_BIT 0xFFF
#define DAC_SETTINGS 0x3000

void GPIO_init(void);
void TIM2_init(void);
void TIM2_IRQHandler(void);
void DAC_write(uint16_t binary_input);
uint16_t DAC_volt_conv(uint16_t voltage_input);
void DAC_init(void);

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	/* USER CODE BEGIN 2 */
	DAC_init();
	GPIO_init();
	TIM2_init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

void GPIO_init(void) {
	/*GPIOC pin 0 set up*/
	// enable GPIO clock
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOCEN);
	// clear Mode bits
	GPIOC->MODER &= ~(GPIO_MODER_MODE1);
	// Set as General Purpose Output
	GPIOC->MODER |= (GPIO_MODER_MODE1_0);
	// set push pull output type
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT1);
	// no PUPD
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1);
	//set to high speed
	GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED1);


	// Enable MCO, select MSI (4 MHz source)
	RCC->CFGR = ((RCC->CFGR & ~(RCC_CFGR_MCOSEL)) | (RCC_CFGR_MCOSEL_0));

	// Configure MCO output on PA8
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER   &= ~(GPIO_MODER_MODE8);		// alternate function mode
	GPIOA->MODER   |=  (2 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT8);		// Push-pull output
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD8);		// no resistor
	GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED8);		// high speed
	GPIOA->AFR[1]  &= ~(GPIO_AFRH_AFSEL8);		// select MCO function

}

void TIM2_IRQHandler(void) {
	//check for CCR1 flag
	GPIOC->ODR |= (GPIO_ODR_OD1);
	//check for CCR flag
	if (TIM2->SR & TIM_SR_CC1IF) {
		//call Dac_write()
		uint16_t input_binary = 13528;
		DAC_write(input_binary);
		//update CCR1 for next reset
		TIM2->CCR1 += CCR1_VALUE + 1;
		//clear CC1 flag
		TIM2->SR &= ~(TIM_SR_CC1IF);
	}
	//turn off pin 1 to end timing of ISR
	GPIOC->ODR &= ~(GPIO_ODR_OD1);

}

void TIM2_init(void) {
	/*Timer setup*/
//enable clock for timer
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
//set to count up direction
	TIM2->CR1 &= ~(TIM_CR1_DIR);
//initialize ARR to ARR_VALUE
	TIM2->ARR = ARR_VALUE;
//initialize CRR1
	TIM2->CCR1 = CCR1_VALUE;
// enable update event interrupt in TIM2
	TIM2->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;
// clear interrupt status register
	TIM2->SR &= ~(TIM_SR_UIF | TIM_SR_CC1IF);
//start timer
	TIM2->CR1 |= TIM_CR1_CEN;
// enable TIM2 interrupt in NVIC
	NVIC->ISER[0] = (1 << TIM2_IRQn);
// enable interrupts globally
	__enable_irq();
}

void DAC_init(void) {
//enable GPIO clk
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE7 | GPIO_MODER_MODE4);
	GPIOA->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE7_1
			| GPIO_MODER_MODE4_1);
//set Chip select to idle at 1
	GPIOA->ODR |= GPIO_ODR_OD4;
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7 | GPIO_OTYPER_OT4);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7
			| GPIO_OSPEEDR_OSPEED4);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD4);

	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7 | GPIO_AFRL_AFSEL4);
	GPIOA->AFR[0] |= ((AF5 << GPIO_AFRL_AFSEL5_Pos)
			| (AF5 << GPIO_AFRL_AFSEL7_Pos) | (AF5 << GPIO_AFRL_AFSEL4_Pos));

	SPI1->CR1 &= ~(SPI_CR1_BR | SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_RXONLY
			| SPI_CR1_LSBFIRST | SPI_CR1_CRCEN);

	SPI1->CR1 |= (SPI_CR1_MSTR);

	SPI1->CR2 |= ((0xF << SPI_CR2_DS_Pos) | SPI_CR2_SSOE | SPI_CR2_NSSP);

	SPI1->CR1 |= SPI_CR1_SPE;

}

uint16_t DAC_volt_conv(uint16_t voltage_input) {
	if (voltage_input > MAX_VOLTAGE) {
		return MAX_TWELVE_BIT;
	}
	if (voltage_input < 0) {
		return 0;
	}

	return DAC_SETTINGS | ((voltage_input * MAX_TWELVE_BIT) / MAX_VOLTAGE);
}

void DAC_write(uint16_t binary_input) {
	GPIOA->ODR &= ~GPIO_ODR_OD4;
	while (!(SPI1->SR & SPI_SR_TXE)) {

	}

	SPI1->DR = (binary_input);
	while (!(SPI1->SR & SPI_SR_TXE))
		;
	GPIOA->ODR |= GPIO_ODR_OD4;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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

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
#include "keypad.h"
#include "table.h"
#define ARR_VALUE 319
#define AF5 5
#define MAX_VOLTAGE 3300
#define MAX_TWELVE_BIT 0xFFF
#define DAC_SETTINGS 0x3000

//wave types
#define SIN 0
#define SQUARE 1
#define SAW 2
#define TRIANGLE 3

volatile uint8_t wave_type = SQUARE;
volatile uint8_t duty_cycle = 50;
volatile uint16_t array_index;
volatile uint8_t frequency_index_increment = 1;
int16_t square_wave[1500];
volatile int8_t keypad_in = 9;
//volatile uint16_t square_table[ARR_VALUE];

void GPIO_init(void);
void TIM2_init(void);
void TIM2_IRQHandler(void);

void generate_waveform(volatile int16_t index);
void set_frequency(int16_t keypad_input);
void generate_square(uint8_t duty_cycle, int16_t *square_table, uint16_t array_size);
void wave_select(int16_t keypad_input);
void set_duty_cycle(uint8_t keypad_input);

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
	generate_square(duty_cycle, square_wave, 1500);
	DAC_init();
	keypad_init();
	//GPIO_init();
	TIM2_init();
	array_index = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		int8_t keypress = NO_PRESS;
		/* USER CODE END WHILE */
		while(keypress == NO_PRESS){
			keypress = keypad_func();
		}
		while (keypad_func() != NO_PRESS);
		keypad_in = keypress;
		set_frequency(keypad_in);
		wave_select(keypad_in);

		if((keypad_in == 0 || keypad_in == ASTERISK || keypad_in == POUND) && wave_type == SQUARE){
			set_duty_cycle(keypad_in);
			generate_square(duty_cycle, square_wave, 1500);
		}


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
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);
	GPIOA->MODER &= ~(GPIO_MODER_MODE8);		// alternate function mode
	GPIOA->MODER |= (2 << GPIO_MODER_MODE8_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT8);		// Push-pull output
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD8);		// no resistor
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8);		// high speed
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8);		// select MCO function

}

void set_frequency(int16_t keypad_input) {
	if (keypad_input == 1) {
		frequency_index_increment = 1;
	} else if (keypad_input == 2) {
		frequency_index_increment = 2;
	} else if (keypad_input == 3) {
		frequency_index_increment = 3;
	} else if (keypad_input == 4) {
		frequency_index_increment = 4;
	} else if (keypad_input == 5) {
		frequency_index_increment = 5;
	}
}

void set_duty_cycle(uint8_t keypad_input){
	if(keypad_input == ASTERISK && duty_cycle > 10){
		duty_cycle -= 10;
	}
	else if(keypad_input == POUND && duty_cycle < 90){
		duty_cycle += 10;
	}
	else if(keypad_input == 0){
		duty_cycle = 50;
	}
}

void wave_select(int16_t keypad_input) {
	if (keypad_input == 6) {
		wave_type = SIN;
	} else if (keypad_input == 7) {
		wave_type = TRIANGLE;
	} else if (keypad_input == 8) {
		wave_type = SAW;
	} else if (keypad_input == 9) {
		wave_type = SQUARE;
	}

}

void generate_square(uint8_t duty_cycle, int16_t *square_table, uint16_t array_size) {
	int high_samples = (duty_cycle * array_size) /100;

	for (int i = 0; i < array_size; i++) {
		if (i < high_samples) {
			square_table[i] = DAC_volt_conv(3000);  // High part of the wave
		} else {
			square_table[i] = DAC_volt_conv(0);  // Low part of the wave
		}
	}
}

void generate_waveform(volatile int16_t index) {
	if (wave_type == SIN) {
		DAC_write(DAC_volt_conv(sine[index]));
	}
	if (wave_type == SQUARE) {
		//generate_square(duty_cycle, square, 1350);
		DAC_write(square_wave[index]);
	}
	if (wave_type == SAW) {
		DAC_write(DAC_volt_conv(saw[index]));
	}
	if (wave_type == TRIANGLE) {
		DAC_write(DAC_volt_conv(triangle[index]));
	}
}

void TIM2_IRQHandler(void) {
	GPIOC->ODR |= (GPIO_ODR_OD1);
//check for ARR flag
	if (TIM2->SR & TIM_SR_UIF) {

		//DAC_write(DAC_volt_conv(0));
		generate_waveform(array_index);
		array_index += frequency_index_increment;
		if (array_index > 1500 - 1) {
			array_index = 0;
		}

		//clear update event interrupt flag
		TIM2->SR &= ~(TIM_SR_UIF);

	}
	GPIOC->ODR &= ~(GPIO_ODR_OD1);
//turn off pin 1 to end timing of ISR


}

void TIM2_init(void) {
	/*Timer setup*/
//enable clock for timer
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
//set to count up direction
	TIM2->CR1 &= ~(TIM_CR1_DIR);
//initialize ARR to ARR_VALUE
	TIM2->ARR = ARR_VALUE;
// enable update event interrupt in TIM2
	TIM2->DIER |= TIM_DIER_UIE;
// clear interrupt status register
	TIM2->SR &= ~(TIM_SR_UIF);
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
  RCC_OscInitStruct.PLL.PLLN = 24;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

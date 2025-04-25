/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * Software made by: Bartosz Wojciechowski
  *
  * software is based on the Arduino RC5 library: https://github.com/guyc/RC5
			 *  RC5 Arduino Library
			 *  Guy Carpenter, Clearwater Software - 2013
			 *  Licensed under the BSD2 license, see LICENSE for details.

  *
  *
  *All text above must be included in any redistribution.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MIN_SHORT  444
#define MAX_SHORT 1333
#define MIN_LONG  1334
#define MAX_LONG  2222


#define EVENT_SHORTSPACE  0
#define EVENT_SHORTPULSE  2
#define EVENT_LONGSPACE   4
#define EVENT_LONGPULSE   6

#define STATE_START1 0
#define STATE_MID1   1
#define STATE_MID0   2
#define STATE_START0 3


#define S2_MASK       0x1000  // 1 bit
#define S2_SHIFT      12
#define TOGGLE_MASK   0x0800  // 1 bit
#define TOGGLE_SHIFT  11
#define ADDRESS_MASK  0x7C0  //  5 bits
#define ADDRESS_SHIFT 6
#define COMMAND_MASK  0x003F //  low 6 bits
#define COMMAND_SHIFT 0


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t micros() {
    uint32_t m;
    uint32_t t;
    uint32_t reload = SysTick->LOAD;
    uint32_t clk_per_us = HAL_RCC_GetHCLKFreq() / 1000000;
    m = HAL_GetTick();   // Read milliseconds
    t = SysTick->VAL;    // Read SysTick counter (decreasing)
    return (m * 1000) + ((reload - t) / clk_per_us);
}


//variables for RC5 functions
const uint8_t trans[] = {0x01,
                               0x91,
                               0x9B,
                               0xFB};

unsigned char state;
uint32_t time0;
GPIO_PinState lastValue;
uint8_t bits;
uint16_t command;

//variables for main code
unsigned char toggle;
unsigned char address;
unsigned char rcommand;
bool  receivedDataTrig = false;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (true == receivedDataTrig)
	  {

	  	char buffer[64];
	  	int length = snprintf(buffer, sizeof(buffer), "RC5:Toggle=%d, Address=%d, Command=%d \r\n", toggle, address, rcommand);
	  	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, length, HAL_MAX_DELAY);
	  receivedDataTrig = false;

	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_IN_Pin */
  GPIO_InitStruct.Pin = IR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void RC5reset()
{
    state = STATE_MID1;
    bits = 1;  // bit counter
    command = 1;
    time0 = micros();
}


void RC5decodeEvent(uint8_t event) //update command
{
    uint8_t newState = (trans[state]>>event) & 0x3;
    if (newState==state) {
        RC5reset();
    } else {
        state = newState;
        if (newState == STATE_MID0) {
            command = (command<<1)+0;
            bits++;
        } else if (newState == STATE_MID1) {
            command = (command<<1)+1;
            bits++;
        }
    }
}

void RC5decodePulse(GPIO_PinState signal, uint32_t period) //determine the event
{
    if (period >= MIN_SHORT && period <= MAX_SHORT) {
    	RC5decodeEvent(signal ? EVENT_SHORTPULSE : EVENT_SHORTSPACE);
    } else if (period >= MIN_LONG && period <= MAX_LONG) {
    	RC5decodeEvent(signal ? EVENT_LONGPULSE : EVENT_LONGSPACE);
    } else {
        // time period out of range, reset
    	RC5reset();
    }
}


bool RC5read1(uint16_t *message) //read RC5
{

	GPIO_PinState value = HAL_GPIO_ReadPin(IR_IN_GPIO_Port, IR_IN_Pin); //adjust to your pin name!!!

    if (value != lastValue) {
    	uint32_t time1 = micros();
    	uint32_t elapsed = time1-time0;
        time0 = time1;
        lastValue = value;
        RC5decodePulse(value, elapsed);
    }

    if (bits == 14) {
        *message = command;
        command = 0;
        bits = 0;
        return 1;
    } else {
        return 0;
    }
}

bool RC5read(unsigned char *toggle, unsigned char *address, unsigned char *command) //read RC5 and update toggle address and command
{
    uint16_t message;
    if (RC5read1(&message)) {
        *toggle  = (message & TOGGLE_MASK ) >> TOGGLE_SHIFT;
        *address = (message & ADDRESS_MASK) >> ADDRESS_SHIFT;

        // Support for extended RC5:
        // to get extended command, invert S2 and shift into command's 7th bit
        uint16_t extended;
        extended = (~message & S2_MASK) >> (S2_SHIFT - 6);
        *command = ((message & COMMAND_MASK) >> COMMAND_SHIFT) | extended;

        return 1;
    } else {
        return 0;
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(RC5read(&toggle, &address, &rcommand))
	{
		receivedDataTrig = true;
	}

}


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

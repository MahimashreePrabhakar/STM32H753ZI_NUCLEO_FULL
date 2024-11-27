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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
#define PWM_DUTY_CYCLE 128 // 50% duty cycle
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint8_t dataReceived = 0;
uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile bool lightBarrierTriggered = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void processCommand(char *command);
void startExtruderMotors(void);
void startFeederAndLEDs(void);
void stopAllOperations(void);
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
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/


     /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Start UART reception in interrupt mode
  HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer, RX_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (dataReceived) {
      processCommand((char *)rxBuffer);
      dataReceived = 0;
    }

    if (lightBarrierTriggered) {
      startFeederAndLEDs();
      lightBarrierTriggered = false; // Reset trigger
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
void processCommand(char *command)
{
  if (strcmp(command, "START") == 0)
  {
    startExtruderMotors();
  }
  else if (strcmp(command, "STOP") == 0)
  {
    stopAllOperations();
  }
}

void startExtruderMotors(void)
{
  // Enable motors by ensuring enable pins are low
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // EXTRUDER_1_EN_PIN
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // EXTRUDER_2_EN_PIN

  // Set PWM duty cycle
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM_DUTY_CYCLE); // PC6 Extruder 1 pwm
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_DUTY_CYCLE); // PB8 Extruder 2 pwm
}

void startFeederAndLEDs(void)
{
  // Set PWM duty cycle for feeder and LEDs
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, PWM_DUTY_CYCLE); // PB15 Feeder 2 pwm
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_DUTY_CYCLE); // PB9 Feeder 1 pwm
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_DUTY_CYCLE); // PA0 L3 pwm
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM_DUTY_CYCLE); // PB0 L4 pwm
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM_DUTY_CYCLE); // PB10 L1 pwm
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_DUTY_CYCLE); // PB11 L2 pwm
}

void stopAllOperations(void)
{
  // Stop all PWM outputs
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // PC6 Extruder 1 pwm
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // PB8 Extruder 2 pwm
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0); // PB15 Feeder 2 pwm
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // PB9 Feeder 1 pwm
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // PA0 L3 pwm
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); // PB0 L4 pwm
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // PB10 L1 pwm
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0); // PB11 L2 pwm

  // Ensure enable pins remain low to keep motors disabled
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // EXTRUDER_1_EN_PIN
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // EXTRUDER_2_EN_PIN
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    if (rxBuffer[0] == '\n' || rxBuffer[0] == '\r')
    {
      rxBuffer[0] = '\0'; // Null-terminate the string
      dataReceived = 1;   // Signal that a command is ready to process
    }
    // Restart UART reception
    HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer, RX_BUFFER_SIZE);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == LB1_OUT_Pin || GPIO_Pin == LB2_OUT_Pin || GPIO_Pin == LB3_OUT_Pin || GPIO_Pin == LB4_OUT_Pin)
  {
    lightBarrierTriggered = true;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

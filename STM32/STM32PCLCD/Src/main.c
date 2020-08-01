/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_GPIO GPIOA
#define LCD_STB GPIO_PIN_0
#define LCD_CLK GPIO_PIN_1
#define LCD_DIO GPIO_PIN_2

#define SERIAL_START_FLAG 0xa0
#define SERIAL_END_FLAG 0x0a
#define SERIAL_RECV_COUNTER_MAX 15 //0xa0 , (mem map) , CMD , 0x0a//
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t SBUFSingel = 0;
uint8_t SBUF[SERIAL_RECV_COUNTER_MAX] = {0};

uint8_t SerialAllowRecv = 0;
uint8_t SerialAllowProcess = 0;

uint8_t SerialRecvCounter = 0;

const uint8_t MEM_MAP[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08, 0x0a, 0x0c, 0x0e, 0x0f};

uint8_t df = 0x0d;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SerialPKGProc(uint8_t *Buf);

//========LCD DEVER=========//
//low
void GPIO_init();
void STB_Clear();
void sendBytes(uint8_t res);
//mid
void sendCommand(uint8_t res);
void LCD_memset(uint8_t adr, uint8_t dat);

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

      /* USER CODE BEGIN SysInit */

      /* USER CODE END SysInit */

      /* Initialize all configured peripherals */
      MX_GPIO_Init();
      MX_USART1_UART_Init();
      MX_TIM3_Init();
      /* USER CODE BEGIN 2 */
      GPIO_init();
      sendCommand(0x44);
      sendCommand(0x88);
      HAL_UART_Receive_IT(&huart1, &SBUFSingel, 1);
      /* USER CODE END 2 */

      /* Infinite loop */
      /* USER CODE BEGIN WHILE */
      while (1)
      {
            HAL_Delay(1);

            if (SerialAllowProcess == 1)
            {
                  SerialPKGProc(SBUF);
                  HAL_UART_Transmit(&huart1, (uint8_t *)&df, 1, 0xffff);
                  SerialAllowProcess = 0;
                  //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,1);
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,1);
                  HAL_Delay(1);
                  //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,0);
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,0);
            }

            /* USER CODE END WHILE */

            /* USER CODE BEGIN 3 */
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
      RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

      /** Initializes the CPU, AHB and APB busses clocks
  */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
      RCC_OscInitStruct.HSIState = RCC_HSI_ON;
      RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
      RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
      RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
      RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
            Error_Handler();
      }
      /** Initializes the CPU, AHB and APB busses clocks
  */
      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
      {
            Error_Handler();
      }
      PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
      PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
      if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
      {
            Error_Handler();
      }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
      if (huart->Instance == USART1)
      {
            if (SerialAllowRecv == 0)
            {
                  if (SBUFSingel == SERIAL_START_FLAG)
                  {
                        SerialAllowRecv = 1;
                  } //SERIAL_START_FLAG
            }

            if (SerialAllowRecv == 1)
            {
                  SBUF[SerialRecvCounter] = SBUFSingel;
                  SerialRecvCounter++;

                  if (SerialRecvCounter == SERIAL_RECV_COUNTER_MAX)
                  {
                        SerialRecvCounter = 0;
                        SerialAllowRecv = 0;
                        if (SBUF[(SERIAL_RECV_COUNTER_MAX - 1)] == SERIAL_END_FLAG)
                        {

                              if (SerialAllowProcess == 0)
                              {
                                    SerialAllowProcess = 1;
                              }

                        } //SERIAL_END_FLAG

                  } //SERIAL_RECV_COUNTER_MAX

            } //SerialAllowRecv

            HAL_UART_Receive_IT(&huart1, &SBUFSingel, 1);
      }
}

void SerialPKGProc(uint8_t *Buf)
{
      if (Buf[0] == SERIAL_START_FLAG && Buf[(SERIAL_RECV_COUNTER_MAX - 1)] == SERIAL_END_FLAG)
      {
            /*for (uint8_t i = 0; i < SERIAL_RECV_COUNTER_MAX; i++)
            {
                  printf("0x%x,", Buf[i]);
            }
            printf("\r\n");*/

            for (uint8_t i = 0; i < 12; i++)
            {
                  //printf("mem %d -> 0x%x\r\n", MEM_MAP[i], Buf[i + 1]);
                  LCD_memset(MEM_MAP[i], Buf[i + 1]);
            }

            if (Buf[(SERIAL_RECV_COUNTER_MAX - 2)] != 0)
            {
                  //printf("cmd : 0x%x\r\n", Buf[(SERIAL_RECV_COUNTER_MAX - 2)]);
                  sendCommand(Buf[(SERIAL_RECV_COUNTER_MAX - 2)]);
            }

            memset((void *)SBUF, 0x00, SERIAL_RECV_COUNTER_MAX);
      }
}

void GPIO_init()
{
      HAL_GPIO_WritePin(LCD_GPIO, LCD_STB, 0);
      HAL_GPIO_WritePin(LCD_GPIO, LCD_CLK, 1);
      HAL_GPIO_WritePin(LCD_GPIO, LCD_DIO, 0);
}

void STB_Clear()
{
      delayMicroseconds(10);
      HAL_GPIO_WritePin(LCD_GPIO, LCD_STB, 1);
      delayMicroseconds(10);
      HAL_GPIO_WritePin(LCD_GPIO, LCD_STB, 0);
      delayMicroseconds(10);
}

void sendBytes(uint8_t res)
{
      uint8_t tmp = res;
      uint8_t outSta = 0;
      for (uint8_t i = 0; i < 8; ++i)
      {
            outSta = tmp & 0x01;

            HAL_GPIO_WritePin(LCD_GPIO, LCD_CLK, 0);
            delayMicroseconds(10);
            HAL_GPIO_WritePin(LCD_GPIO, LCD_DIO, outSta);
            delayMicroseconds(10);
            HAL_GPIO_WritePin(LCD_GPIO, LCD_CLK, 1);
            delayMicroseconds(10);

            tmp = tmp >> 1;
      }
}

void sendCommand(uint8_t res)
{
      STB_Clear();
      sendBytes(res);
}

void LCD_memset(uint8_t adr, uint8_t dat)
{
      uint8_t baseAdressCommand = 0xc0;
      uint8_t AdressLow4B = adr & 0x0f;
      baseAdressCommand = baseAdressCommand | AdressLow4B;
      STB_Clear();
      sendBytes(baseAdressCommand);
      sendBytes(dat);
}

int __io_putchar(int ch)
{

      HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);

      return ch;
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

      /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
      /* USER CODE BEGIN 6 */
      /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
      /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

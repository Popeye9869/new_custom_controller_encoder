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
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_def.h"
#include "stm32g0xx_hal_uart.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum {
  reset = 0,
  master,
  slave
} mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_LENGTH 10
#define MASTER_SENT_TIME_OUT 30
#define SLAVE_SENT_TIME_OUT 30
#define SLAVE_RECEIVE_TIME_OUT 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RawData[2] ={0};//原始数据
uint16_t RawAngle = 0;//原始角度值
uint8_t Start_pData[8] = {0};
uint16_t Data[DATA_LENGTH] = {0};//包头0x55AA，DATA_LENGTH-2个数据，最后一位为传感器链路位置
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CycDataProc(void);
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  mode = master;

  // HAL_UART_Receive_IT(&huart1, Start_pData, 8);

  // HAL_Delay(1000);

  // HAL_UART_Transmit(&huart2, (uint8_t *)"Start\r\n", 8, HAL_MAX_DELAY);//发送信号以确定链路起始

  // HAL_Delay(1000);
  // HAL_UART_AbortReceive_IT(&huart1);

  if(mode == reset) //初始化-判断是否为主机
    mode = master;

  if(mode == master)
  {
    HAL_TIM_Base_Start_IT(&htim1); //主机启动定时器中断
  }

  if(mode == slave)
  {
    HAL_UART_Receive_DMA(&huart1, (uint8_t *)Data, DATA_LENGTH*2); //从机启动DMA接收
  }

 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    HAL_I2C_Mem_Read(&hi2c2, 0x36<<1,
                    0x0C, I2C_MEMADD_SIZE_8BIT,
                    RawData, 2, 100);
    RawAngle = (RawData[0]<<8 | RawData[1]);
    HAL_Delay(5);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void CycDataProc(void)
{
  // 优化：一次性定位帧头并旋转缓冲区，避免O(n^2)逐字节前移；修正小端字节序匹配
  uint8_t *p = (uint8_t *)Data;
  const size_t len = DATA_LENGTH * 2U; // 字节数
  size_t i = 0;

  // 查找小端序的帧头 0x55AA -> 字节序列 0xAA 0x55
  while (i + 1 < len) {
    if (p[i] == 0xAA && p[i + 1] == 0x55) {
      break;
    }
    ++i;
  }

  // 已对齐或未找到帧头则返回
  if (i == 0 || i + 1 >= len) {
    return;
  }

  // 旋转：把 [i, len) 移到前面，把 [0, i) 追加到末尾
  uint8_t tmp[DATA_LENGTH * 2U]; // 最大20字节
  memcpy(tmp, p, i);                 // 保存前 i 字节
  memmove(p, p + i, len - i);        // 前移剩余数据
  memcpy(p + (len - i), tmp, i);     // 追加原前缀
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1 && mode == reset) //初始化-判断是否为从机
  {
    if(strcmp((char *)Start_pData,"Start\r\n")==0)
      mode = slave;
  }

  if(huart->Instance == USART1 && mode == slave) //从机接收完成回调
  {
    CycDataProc(); //循环数据处理
    Data[DATA_LENGTH - 1]++ ; //标记数据链路位置为从机位置
    Data[Data[DATA_LENGTH - 1]] = RawAngle; //存储数据
    HAL_UART_Transmit(&huart2, (uint8_t *)Data, DATA_LENGTH * 2, SLAVE_SENT_TIME_OUT);
  }


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1) //主机定时器中断
  {
    Data[0] = 0x55AA; //存储数据
    Data[1] = RawAngle;
    Data[DATA_LENGTH - 1] = 1; //标记数据链路位置为主机位置
    HAL_UART_Transmit(&huart2, (uint8_t *)Data, DATA_LENGTH * 2, MASTER_SENT_TIME_OUT);
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
#ifdef USE_FULL_ASSERT
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

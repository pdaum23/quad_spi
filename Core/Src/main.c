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
#include "quadspi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25q_mem.h"
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
int main(void)
{

  /* USER CODE BEGIN 1 */
  //int i;
  uint8_t testBuf[256];
  // make test data
  uint8_t byte = 0x65;
  uint8_t byte_read = 0;
  uint8_t in_page_shift = 0;
  uint8_t page_number = 0;
  //uint8_t testData[2] = {0xAA, 0x55};
  //uint8_t testDataRead[2] = {0x00, 0x00};
  // make example structure
  struct STR {
    uint8_t abc;
    uint32_t bca;
    char str[4];
    float gg;
  } _str, _str2;
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
  MX_QUADSPI_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  W25Q_Init();		 // init the chip
  W25Q_EraseSector(0); // Erase sector 0 for write read test

  // read data
  W25Q_ReadByte(&byte_read, in_page_shift, page_number);
  // write data
  W25Q_ProgramByte(byte, in_page_shift, page_number);
  W25Q_ReadByte(&byte_read, in_page_shift, page_number);

  //W25Q_ProgramData(testData, 2, in_page_shift, ++page_number);
  //W25Q_ReadData(testDataRead, 2, in_page_shift, page_number);
  // fill instance
  _str.abc = 0x20;
  _str.bca = 0x3F3F4A;
  _str.str[0] = 'a';
  _str.str[1] = 'b';
  _str.str[2] = 'c';
  _str.str[3] = '\0';
  _str.gg = 0.658;

  u16_t len = sizeof(_str); // length of structure in bytes

  // program structure
  W25Q_ProgramData((u8_t*) &_str, len, ++in_page_shift, page_number);
  // read structure to another instance
  W25Q_ReadData((u8_t*) &_str2, len, in_page_shift, page_number);

  W25Q_ReadRaw(testBuf, 256, page_number*MEM_PAGE_SIZE);
  page_number++;
  W25Q_ReadRaw(testBuf, 256, page_number*MEM_PAGE_SIZE);
  memset(testBuf, 0xAA, 256);
  W25Q_ProgramRaw(testBuf, 256, page_number*MEM_PAGE_SIZE);
  memset(testBuf, 0x55, 256);
  W25Q_ReadRaw(testBuf, 256, page_number*MEM_PAGE_SIZE);
  memset(testBuf, 0x55, 256);
  page_number++;
  W25Q_ReadRaw(testBuf, 256, page_number*MEM_PAGE_SIZE);

  memset(testBuf, 0x55, 256);
  W25Q_ReadRaw(testBuf, 256, 0);
  QSPI_AutoPollingMemReady();
  //QSPI_WriteEnable();
  CSP_QSPI_EnableMemoryMappedMode();
  memset(testBuf, 0x55, 256);
  memcpy(testBuf, (uint8_t *) 0x90000000, 256);
  QSPI_ResetChip();
  W25Q_Init();
  memset(testBuf, 0x55, 256);
  W25Q_ReadRaw(testBuf, 256, 0);


  __NOP();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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

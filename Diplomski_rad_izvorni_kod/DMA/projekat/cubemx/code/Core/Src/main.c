/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
extern DMA_HandleTypeDef hdma_usart1_tx;
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
	/*
	 * U ovom primeru bice demonstrirana upotreba DMA kontrolera za prenos jedne
	 * velike poruke iz memorije naseg mikrokontrolera na USART periferiju.
	 * Poruka ce biti odstampana na virtuelnom terminalu a LE Dioda ce biti
	 * Togglovana u prekidnoj rutini kada se kompletan prenos obavi.
	 * Postoji mogucnost da se generise prekid i na polovini i na kraju transfera
	 * podataka pomocu registrovanja odgovarajucih CallBack funkcija za DMA periferiju.
	 * Odgovarajuca prekidna rutina definisana je u stm32fxx_it.c fajlu.
	 */
	char msg[] ="Ova poruka je preneta putem DMA kontrolera transferom iz memorije"
				" na periferiju USART! LE dioda se Toggluje u prekidnoj rutini nakon"
				" zavrsenog prenosa podataka!\r";
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
   * Nakon konfiguracije odgovarajucih periferija u beskonacnoj petlji cemo na svakih
   * sekund prenositi odgovarajucu poruku pomocu DMA kontrolera. Da bismo omogucili
   * prenos podataka na USART periferiju putem DMA kontrolera neophodno je da u
   * kontrolnom registru USART periferije postavimo odgovarajuci bit USART_CR3_DMAT.
   * Alternativno, mogli smo da pozovemo funkciju HAL_UART_Transmit_DMA(huart, pData, Size)
   * koja bi samostalno postavila odgovarajuci bit i izvrsila prenos podataka. Izabrana je
   * prva opcija da bi se bolje demonstrirala upotreba DMA kontrolera pozivom funkcije
   * HAL_DMA_Start_IT kojoj se kao parametri prosledjuju: rucka na strukturu u kojoj su
   * konfigurisani svi neophodni parametri za prenos iz memorije na USART periferiju,
   * pocetna adresa sa koje ce se slati podaci, pocenta adresa na koju ce se smestati podaci
   * kao i broj podataka koji se prenosi. U prekidnoj rutini cemo resetovati odgovarajuci bit
   * u kontrolnom registru USART periferije da bi se ona mogla koristiti na drugim mestima
   * iz programskog koda, ukoliko bi bilo potrebe za tim.
   */
	while (1) {
		huart1.Instance->CR3 |= USART_CR3_DMAT;
		HAL_DMA_Start_IT(&hdma_usart1_tx, (uint32_t) msg,
				(uint32_t) &huart1.Instance->DR, strlen(msg));
		HAL_Delay(1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

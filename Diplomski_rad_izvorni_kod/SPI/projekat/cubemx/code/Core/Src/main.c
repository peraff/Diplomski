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
#include "spi.h"
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flag;
extern DMA_HandleTypeDef hdma_usart1_tx;

//Komande za komunikaciju sa mikrokontrolerom EEPROM uredjaja.
//Odgovarajuca dokumenacija u pdfu je dostupna u direktorijumu projekat/cubemx/code
const uint8_t WRITE = 0x02;
const uint8_t READ = 0x03;
const uint8_t WREN = 0x06;
const uint8_t RDSR = 0x05;
const uint8_t WRSR = 0x01;

void SystemClock_Config(void);

int main(void) {
	//private variables
	char spi_buff[20];
	char uart_buff[70];
	char init_data[60];
	uint8_t addr;
	uint8_t wip;
	uint8_t lenn;

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
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* CS(Chip Select) signal sluzi da gazda na magistrali odabere sa kojim
	 * slugom ce komunicirati u slucaju da postoji vise sluga. Ovaj signal treba
	 * drzati na visokom naponskom nivou uvek, osim u slucaju kada se vrsi
	 * komunikacija sa odgovarajucim slugom sa kojim je gazda povezan putem ovog pina.
	 * U nasem primeru postoji samo jedan sluga na magistrali i to je EEPROM uredjaj,
	 * pa ce se signal CS(signal sa pina 4 porta A) svaki put prvo obarati
	 * na nizak naponski nivo, pre bilo kakve komunikacije, dok ce se na kraju
	 * komunikacije ovaj signal vracati na visok naponski nivo.
	 *
	 */

	//CS postavljen visoko
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//Slanje write-enable komande (omogucava se upis podataka na uredjaj -
	// inicijalno uredjaj je u read only rezimu).
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WREN, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//Ocitavanje statusnog registra - saljemo odgovarajucu komandu nakon koje
	// primamo jedan bajt podataka koji predstavlja sadrzaj statusnog registra
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &RDSR, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t*) spi_buff, sizeof(uint8_t),
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//fleg se postavlja pre prenosa podataka putem DMA kontrolera, a u prekidnoj
	// rutini se resetuje, nakon sto su svi podaci uspesno preneti na periferiju
	// Odgovarajuca prekidna rutina nalazi se u fajlu stm32f1xx_it.c
	flag = 1;
	lenn = sprintf((char*) uart_buff,
			"Statusni registar(Postavljen WEL bit): 0x%02x",
			(unsigned int) spi_buff[0]);
	huart1.Instance->CR3 |= USART_CR3_DMAT;
	HAL_DMA_Start_IT(&hdma_usart1_tx, (uint32_t) uart_buff,
			(uint32_t) &huart1.Instance->DR, lenn);

	//Podaci koje cemo upisati na uredjaj.
	spi_buff[0] = 'x';
	spi_buff[1] = 'y';
	spi_buff[2] = 'z';

	//Adresa na koju se upisuju podaci
	addr = 0x10;

	/*Upisivanje 3 bajta podataka pocev od zadate adrese. Prvo se salje WRITE komanda
	 koju sledi adresa na koju ce biti smesteni podaci (obratiti paznju na dokumentaciju
	 uredjaja, najvisi bit adrese zapisan je u okviru komande!(Pomocu ove WRITE komande
	 salju se adrese gde je najvise bit uvek fiksno 0!). Nakon adrese salju se redom podaci
	 do 16 bajtova podataka koji ce biti upisani u odgovarajucu stranicu memorije
	 (ako se dodje do kraja stranice, preostali podaci se upisuju pocev od prve adrese date stranice.
	 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &WRITE, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) spi_buff, 3 * sizeof(uint8_t),
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//Ciscenje bafera da bismo u njega primili podatke
	spi_buff[0] = 0;
	spi_buff[1] = 0;
	spi_buff[2] = 0;

	//Uposleno cekanje da se zavrsi upis na uredjaju! U slucaju da se izostavi ova petlja
	// uredjaj bi nas obavestio o gresci pri citanju i vratio bi zateknute vrednosti.
	do {
		//Ocitavanje Statusnog registra
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &RDSR, sizeof(uint8_t),
		HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, (uint8_t*) spi_buff, sizeof(uint8_t),
		HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		//Maskiramo najnizi bit statusnog registra koji indikuje da li je trenutno upis
		// podataka u toku. Kada se upis zavrsi ovaj bit ce biti postavljen na 0.
		wip = spi_buff[0] & 0x01;
	} while (wip);

	// GRESKA U SIMULATORU! Da bismo procitali podatke koje smo upisali na adresu 0x10,
	// pri operaciji citanja moramo zadati za jedan nizu vrednost adrese.
	addr = 0x0F;

	//Citanje podataka sa uredjaja. Salje se READ komanda pracena adresom sa koje se
	//citaju podaci(adresa umanjena za 1!). Nakon koje se blokiramo dok ne primimo odgovarajuce podatke
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &READ, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t*) spi_buff, 3 * sizeof(uint8_t),
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//Cekamo dok se ne zavrsi prethodni prenos podataka putem DMA kontrolera,
	// pre nego sto zapocnemo novi. Fleg ce biti resetovan u prekidnoj rutini DMA kontrolera.
	while (flag) {
	}
	flag = 1;
	lenn = sprintf((char*) uart_buff, "\rUpisani podaci na uredjaj: %c%c%c\r",
			spi_buff[0], spi_buff[1], spi_buff[2]);
	huart1.Instance->CR3 |= USART_CR3_DMAT;
	HAL_DMA_Start_IT(&hdma_usart1_tx, (uint32_t) uart_buff,
			(uint32_t) &huart1.Instance->DR, lenn);

	//Ponovo cemo ocitati statusni registar nakon sto je izvrsena operacija upisa
	// WEL bit je resetovan(drugi najnizi bit) i uredjaj je vracen u read only mod.
	// Neophodno je ponovo poslati WREN komandu pre narednog upisa na uredjaj!
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &RDSR, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t*) spi_buff, sizeof(uint8_t),
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//cekamo da se zavrsi prethodni upis pre nego sto mozemo zapoceti novi
	while (flag) {
	}
	flag = 1;
	lenn = sprintf((char*) uart_buff,
			"\rStatusni registar(Nakon upisa - resetovan WEL bit): 0x%02x ",
			(unsigned int) spi_buff[0]);
	huart1.Instance->CR3 |= USART_CR3_DMAT;
	HAL_DMA_Start_IT(&hdma_usart1_tx, (uint32_t) uart_buff,
			(uint32_t) &huart1.Instance->DR, lenn);

	while (flag) {
	}
	flag = 1;

	/* Dodatno, moguce je EEPROM uredjaj inicijalizovati odgovarajucim pocetnim vrednostima
	 * putem binarnog fajla. U direktorijumu projekat/proteus dat je fajl EEPROM_INIT.bin
	 * U ovom fajlu dovoljno je navesti ASCII vrednosti koje ce biti ucitane pocev od
	 * adrese 0x00 u memoriju uredjaja. Desnim klikom na uredjaj u simulatoru otvara se
	 * meni iz kojeg treba odabrati opciju Edit Properties. U polje polje Initial
	 * memory contents treba navesti putanju do binarnog fajla sa pocetnim sadrzajem memorije.
	 * (Moze se koristi opcija pretrazivanja klikom na slicicu foldera.)
	 *
	 * NAPOMENA (VAZNO!!!) : Obavezno nakon bilo koje promene ili ucitavanja inicijalizacionog
	 * fajla u Proteus simulatoru iz glavnog menija izabrati Debug->Reset Persistent Model Data
	 * da bi bilo koja promena kao i ucitavanje fajla bili vidljivi!
	 *
	 * U primeru ispod su procitani podaci koji su smesteni u ovom fajlu, s tim sto
	 * prvi karakter ne moze biti procitan usled greske kod operacije citanja koja je navedena ranije.
	 *
	 */

	spi_buff[0] = 0;
	spi_buff[1] = 0;
	spi_buff[2] = 0;

	addr = 0x00;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &READ, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, sizeof(uint8_t), HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t*) spi_buff, 4 * sizeof(uint8_t),
	HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	lenn = sprintf((char*) init_data,
			"\rProcitani inicijalni podaci sa uredjaja: %c%c%c%c\r",
			spi_buff[0], spi_buff[1], spi_buff[2], spi_buff[3]);
	huart1.Instance->CR3 |= USART_CR3_DMAT;
	HAL_DMA_Start_IT(&hdma_usart1_tx, (uint32_t) init_data,
			(uint32_t) &huart1.Instance->DR, lenn);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

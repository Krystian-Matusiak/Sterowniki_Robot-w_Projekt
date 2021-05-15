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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#define CONV_NUM 3
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
volatile uint16_t czuj1;
volatile uint16_t czuj2;
volatile uint16_t czuj3;
volatile uint16_t czuj4;
volatile uint16_t czuj5;
volatile uint16_t czuj6;

volatile uint8_t Czujniki[6];

volatile uint8_t adc1[3];
volatile uint8_t adc2[3];


//////////////////////////////////////////////////////////////////////////////
// Funkcje ustawiające kierunek ruchu pierwszego silnika

void setDIR1_M1(){
	HAL_GPIO_WritePin(DIR1_M1_GPIO_Port, DIR1_M1_Pin, SET);
}
void resetDIR1_M1(){
	HAL_GPIO_WritePin(DIR1_M1_GPIO_Port, DIR1_M1_Pin, RESET);
}
void setDIR2_M1(){
	HAL_GPIO_WritePin(DIR2_M1_GPIO_Port, DIR2_M1_Pin , SET);
}
void resetDIR2_M1(){
	HAL_GPIO_WritePin(DIR2_M1_GPIO_Port, DIR2_M1_Pin, RESET);
}

void setM1_Forward(){
	resetDIR1_M1();
	setDIR2_M1();
}

void setM1_Backward(){
	setDIR1_M1();
	resetDIR2_M1();
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Funkcje ustawiające kierunek ruchu drugiego silnika

void setDIR1_M2(){
	HAL_GPIO_WritePin(DIR1_M2_GPIO_Port, DIR1_M2_Pin, SET);
}
void resetDIR1_M2(){
	HAL_GPIO_WritePin(DIR1_M2_GPIO_Port,DIR1_M2_Pin, RESET);
}
void setDIR2_M2(){
	HAL_GPIO_WritePin(DIR2_M2_GPIO_Port, DIR2_M2_Pin, SET);
}
void resetDIR2_M2(){
	HAL_GPIO_WritePin(DIR2_M2_GPIO_Port, DIR2_M2_Pin, RESET);
}

void setM2_Forward(){
	resetDIR1_M2();
	setDIR2_M2();
}

void setM2_Backward(){
	setDIR1_M2();
	resetDIR2_M2();
}
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Funkcje ustawiające wypełnienie PWM'ów obu silników

void setPWM_Motor1( int wypelnienie ){
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, wypelnienie);
}

void setPWM_Motor2(int wypelnienie){
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, wypelnienie);
}

//////////////////////////////////////////////////////////////////////////////

void setSTBY(){
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, SET);
}

/*
int _write(int file, char *ptr, int len){
	  HAL_UART_Transmit( &huart1, ptr, len,50 );
	  return len;
}*/


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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, adc1, CONV_NUM);
  HAL_ADC_Start_DMA(&hadc2, adc2, CONV_NUM);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  setSTBY();


  /*******************************/

  setM1_Forward();
  setM2_Forward();

  int prev_error=0;
  int error=0;
  int ilosc_wykryc=0;
  int waga[] = { -10 , -7 , -5 , 5 , 7 , 10 };

  int Kp = 2;
  int Kd = 1;

  uint8_t Test1;
  uint8_t Test2;
  uint8_t Test3;
  uint8_t Test4;
  uint8_t Test5;
  uint8_t Test6;

  int rozniczka=0;
  int regulacja=0;

  /* Wartości, dla których silniki kręcą się z tę samą szybkością*/
  int setpoint_M1 = 40;
  int setpoint_M2 = 33;

  int prog = 150;


  while (1)
  {
	  //  Czujnik 1
	  if(adc1[0] > prog)
		  Czujniki[0]=1;
	  else
		  Czujniki[0]=0;

	  //  Czujnik 2
	  if(adc1[1] > prog)
		  Czujniki[1]=1;
	  else
		  Czujniki[1]=0;

	  //  Czujnik 3
	  if(adc1[2] > prog)
		  Czujniki[2]=1;
	  else
		  Czujniki[2]=0;

	  //  Czujnik 4
	  if(adc2[0] > prog)
		  Czujniki[3]=1;
	  else
		  Czujniki[3]=0;

	  //  Czujnik 5
	  if(adc2[1] > prog)
		  Czujniki[4]=1;
	  else
		  Czujniki[4]=0;

	  //  Czujnik 6
	  if(adc2[2] > prog)
		  Czujniki[5]=1;
	  else
		  Czujniki[5]=0;


	  Test1 = adc1[0];
	  Test2 = adc1[1];
	  Test3 = adc1[2];
	  Test4 = adc2[0];
	  Test5 = adc2[1];
	  Test6 = adc2[2];


	  for(int n=0; n<6 ; n++ ){
		  error += Czujniki[n]*waga[n];
		  ilosc_wykryc += Czujniki[n];
	  }

	  if(ilosc_wykryc != 0 ){
		  error /= ilosc_wykryc;
		  prev_error = error;
	  }
	  else{
		  if(prev_error < -7)
			  error = -12;
		  else if(prev_error > 7)
			  error= 12;
		  else
			  error=0;
	  }

	  rozniczka = error - prev_error;
	  prev_error = error;
	  regulacja = Kp*error + Kd*rozniczka;

	  setPWM_Motor1( setpoint_M1 - regulacja );
	  setPWM_Motor2( setpoint_M2 + regulacja );

	  ilosc_wykryc=0;
	  error=0;

	  HAL_ADC_Start_DMA(&hadc1, adc1, CONV_NUM);
	  HAL_ADC_Start_DMA(&hadc2, adc2, CONV_NUM);
	  HAL_Delay(20);


		/*
	      printf("Czuj1 %d\r\n" , adc1[0]);
		  printf("Czuj2 %d\r\n" , adc1[1]);
		  printf("Czuj3 %d\r\n" , adc1[2]);
		  printf("Czuj4 %d\r\n" , adc2[0]);
		  printf("Czuj5 %d\r\n" , adc2[1]);
		  printf("Czuj6 %d\r\n\n" , adc2[2]);

		  */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

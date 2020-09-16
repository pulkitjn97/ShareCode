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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include "MPU6050_LIB.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// seed values
float ax=0.0, ay=100.0, az=200.0;
//int iax=0;
uint8_t bytes_temp[4];

/*
 * @brief Structure to hold and receive all sensor data
 * @param Voltage (V), Current (I), Temperature (t), Speed (S), Throttle (Th),
 * @size (5 x uint16) + (6 x Bool(1)) + (6 x Float(4)) = 34B + 1B (for Bool)
 */

struct TX_Frame{

	//uint16_t V,I,t,S, Th;
	//_Bool _Sys, _HL, _FAIL, _BR, _MODEH, _MODEL;
	float AX, AY, AZ; //GX, GY, GZ;

} TX_Data1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float genXLvals(uint16_t min, uint16_t max)
{
  return (rand() % ((max-min+1)) + min);
}


/*
 * @brief Function to convert and send data over UART
 * @param XL_Data, UART Handle Typedef
 */
void sendStruct(struct TX_Frame* data, UART_HandleTypeDef* huart){
  // UART can only send unsigned int
  // Thus we need to convert our struct data
  char buffer[sizeof(data)]; // Create a char buffer of right size

  // Copy the data to buffer
  memcpy(buffer, &data, sizeof(data)); // Copy and convert the data
  // Ideally buffer will be 12B long, 4B for each axes data
  // Now we can finally send this data
  HAL_UART_Transmit(huart, (uint8_t*) buffer, sizeof(buffer), 50);
  // The last param is timeout duration in ms
}

//Custom write funtion for printing in SWO printf Trace
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //HAL_UART_Transmit(&huart1, (uint8_t*)"initialized \n", strlen("initialized \n"), 500);

  //HAL_Delay(1000);
  //HAL_UART_Transmit(&huart1, (uint8_t*)"MPU6050 \n", strlen("MPU6050 \n"), 500);


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  //MPU6050_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  void float2Bytes(uint8_t* ftoa_bytes_temp,float float_variable)
  {
    union
	{
      float a;
      unsigned char bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++)
    {
		  ftoa_bytes_temp[i] = thing.bytes[i];
	}

  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  // Indicate Start of Sequence with "S"
	  HAL_UART_Transmit(&huart1,"S", 1, 50);

	  //Read Fresh Accelerometer Values
	  //MPU6050_Read_Accel();

	  // Populate the Accelerometer Structure
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
	  float2Bytes(bytes_temp,ax);
	  HAL_UART_Transmit(&huart1,bytes_temp, sizeof(bytes_temp), 50);
	  ax= ax+0.1;
	  float2Bytes(bytes_temp,ay);
	  HAL_UART_Transmit(&huart1,bytes_temp, sizeof(bytes_temp), 50);
	  ay= ay+0.2;
	  float2Bytes(bytes_temp,az);
	  HAL_UART_Transmit(&huart1,bytes_temp, sizeof(bytes_temp), 50);
	  az= az+0.3;

	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);


	  //HAL_UART_Transmit(&huart1, (uint8_t*)"\n", sizeof("\n"), 50);
	  //float2Bytes(bytes_temp, Ay);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)bytes_temp, sizeof(bytes_temp), 50);
	  //float2Bytes(bytes_temp, Az);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)bytes_temp, sizeof(bytes_temp), 50);
	  //TX_Data1.AX = Ax;
	  //c=TX_Data1.AX;
	  //TX_Data1.AY = Ay;
	  //d=TX_Data1.AY;
	  //TX_Data1.AZ = Az;
	  //e=TX_Data1.AZ;

	  // Send the Data over UART
	  //sendStruct(&TX_Data1, &huart1); // huart1 is auto-generated for USART1


	  //float f;
	  //memcpy(&f, &bytes_temp, sizeof(f));
	  //return f;

	  printf ("Data Sent is Ax= %.2f Ay=%.2f Az=%.2f \n", ax, ay, az);
	  //printf ("Data Sent is Ax= %d \n", iax);

	  // Indicate End of Sequence with "Z"
	  //HAL_UART_Transmit(&huart1, (uint8_t*)"b", sizeof("b"), 50);
	  HAL_Delay(500); // Delay for 500ms

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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

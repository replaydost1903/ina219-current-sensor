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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
ina219_config ina219={0};
double Current=0,Shunt_Voltage=0,Bus_Voltage=0,Power=0;
char tx_buff[50]={0};
double buffer[MEDIAN_FILTER_N_VALUE]={0};
double filter_result_current=0,filter_result_bus_voltage=0,filter_result_shunt_voltage=0,filter_result_power=0;
struct ina219_moving_filter mov={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SystemInit();	//FPU Enable
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /*******************************
   * 	INA219 Sensor Testing
   ******************************/
  ina219_selftest();



  /*******************************
   * 	INA219 Init Settings
   ******************************/
  ina219.bus_voltage_range = INA219_BUS_VOLTAGE_RANGE_32V;
  ina219.pga = INA219_GAIN_1;
  ina219.bus_adc_resolution = INA219_BUS_ADC_MODE_12BIT;
  ina219.shunt_adc_resolution = INA219_SHUNT_ADC_MODE_12BIT;
  ina219.operating_mode = INA219_MODE_SHUNT_AND_BUS_CONTINOUS;
  ina219_init(ina219);


  /********************************************
   *	INA219 Calibration Value Setting
   *******************************************/
  ina219_calculate_and_write_calibration_value(ina219);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Bus_Voltage = ina219_read_bus_voltage();
	  Shunt_Voltage = ina219_read_shunt_voltage();
	  Current = ina219_read_shunt_and_bus_current(ina219);
	  Power = ina219_read_bus_power(ina219);


	  /**********************************
	   *  	 average moving filter		*
	   **********************************/

	  /**
	   *  İstenilen veri adedi kadar buffera depola
	   */
	  for(uint32_t count=0;count<__array_length(buffer);count++)
	  {
		  mov.current[count] = ina219_read_shunt_and_bus_current(ina219);
		  mov.bus_voltage[count] = ina219_read_bus_voltage();
		  mov.shunt_voltage[count] = ina219_read_shunt_voltage();
		  mov.power[count] = ina219_read_bus_power(ina219);
	  }

	  /**
	   *  Depoladığın verileri filtreden geçir
	   */
	  filter_result_current = ina219_moving_average_filter(mov.current,__array_length(buffer));
	  filter_result_bus_voltage = ina219_moving_average_filter(mov.bus_voltage,__array_length(buffer));
	  filter_result_shunt_voltage = ina219_moving_average_filter(mov.shunt_voltage,__array_length(buffer));
	  filter_result_power = ina219_moving_average_filter(mov.power,__array_length(buffer));


	  /**
	   *  Filtrelenmiş verileri ve filtrelenmemiş verileri seri porta aktar
	   */
	  HAL_UART_Transmit(&huart2,(uint8_t*)"****************",strlen("****************"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"\n"); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Non-Filter Result Bus Voltage	",strlen("Non-Filter Result Bus Voltage	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f   V\n",Bus_Voltage); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Non-Filter Result Shunt Voltage	",strlen("Non-Filter Result Shunt Voltage	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f  mV\n",Shunt_Voltage); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Non-Filter Result Current     	",strlen("Non-Filter Result Current     	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f  mA\n",Current); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Non-Filter Result Power     	",strlen("Non-Filter Result Power     	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f mW\n",Power); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  sprintf(tx_buff,"\n\n"); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart2,(uint8_t*)"****************",strlen("****************"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"\n"); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Filter Result Bus Voltage	",strlen("Filter Result Bus Voltage	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f   V\n",filter_result_bus_voltage); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Filter Result Shunt Voltage	",strlen("Filter Result Shunt Voltage	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f  mV\n",filter_result_shunt_voltage); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Filter Result Current     	",strlen("Filter Result Current     	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f  mA\n",filter_result_current); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  HAL_UART_Transmit(&huart2,(char*)"Filter Result Power     	",strlen("Filter Result Power     	"),HAL_MAX_DELAY);
	  sprintf(tx_buff,"%f mW\n",filter_result_power); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);

	  sprintf(tx_buff,"\n"); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);
	  HAL_UART_Transmit(&huart2,(uint8_t*)"****************",strlen("****************"),HAL_MAX_DELAY);

	  sprintf(tx_buff,"\n"); // @suppress("Float formatting support")
	  HAL_UART_Transmit(&huart2,(uint8_t*)tx_buff,strlen(tx_buff),HAL_MAX_DELAY);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  hi2c1.Init.ClockSpeed = 400000;
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin ORANGE_LED_Pin RED_LED_Pin BLUE_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|ORANGE_LED_Pin|RED_LED_Pin|BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

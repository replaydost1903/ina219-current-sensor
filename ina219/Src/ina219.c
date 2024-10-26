/**
  ******************************************************************************
  * @file    ina219.c
  * @author  Kubilay Közleme
  * @brief   INA219 current measurement sensor driver
  * @version V1
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  @verbatim
  */


#include "ina219.h"


extern I2C_HandleTypeDef hi2c1;
HAL_StatusTypeDef status;


//helper static functions
static double ina219_calculate_max_expected_current(ina219_config);
static void error_signal(void);
static void complete_signal(void);
static uint16_t ina219_read(uint8_t);
static void ina219_write(uint8_t,uint16_t);



/**
  * @brief ina219 sensor test
  */
HAL_StatusTypeDef ina219_selftest(void)
{
	ina219_power_on_reset();

	status = HAL_I2C_IsDeviceReady(&hi2c1, INA219_WRITE_ADDR, 100, HAL_MAX_DELAY);

	switch(status)
	{
	case HAL_OK:
		complete_signal();
		break;
	case HAL_ERROR:
		error_signal();
		break;
	case HAL_BUSY:
		error_signal();
		break;
	case HAL_TIMEOUT:
		error_signal();
		break;
	default:
		break;
	}
	return status;
}


/**
  * @brief ina219 power on reset test
  */
void ina219_power_on_reset(void)
{
	ina219_write(INA219_CONFIG_REG_ADDR, INA219_RST);
	while((ina219_read(INA219_CONFIG_REG_ADDR) & (INA219_RST)));
}

/**
  * @brief ina219 read register
  */
uint16_t ina219_read(uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(&hi2c1, INA219_READ_ADDR, Register, 1, Value, sizeof(Value), HAL_MAX_DELAY);

	return ((Value[0] << 8) | Value[1]);
}

/**
  * @brief ina219 write register
  */
void ina219_write(uint8_t Register , uint16_t data)
{
	uint8_t addr[2];

	addr[0] = (data >> 8) & 0xff;
	addr[1] = (data >> 0) & 0xff;

	HAL_I2C_Mem_Write(&hi2c1, INA219_WRITE_ADDR, Register , 1, addr, sizeof(addr), HAL_MAX_DELAY);
}

/**
  * @retval HAL status
  */
HAL_StatusTypeDef ina219_init(ina219_config ina219)
{
	uint16_t temp1 = 0 , temp2 = 0;

	temp1 = (ina219.bus_voltage_range << 13U) 		|   \
			(ina219.pga << 11U)				  		|	\
			(ina219.bus_voltage_range << 7U)  		|	\
			(ina219.shunt_adc_resolution << 3U)		|	\
			(ina219.operating_mode << 0U);

	ina219_write(INA219_CONFIG_REG_ADDR, temp1);

	temp2 = ina219_read(INA219_CONFIG_REG_ADDR);

	if(temp1 == temp2)
	{
		return HAL_OK;
	}

	return HAL_ERROR;
}

/**
  * @retval maximum possible current
  */
static double ina219_calculate_max_expected_current(ina219_config ina219)
{			//0,1,2,3
	double max_shunt_voltage=0;

	switch(ina219.pga)
	{
	case 0:
		max_shunt_voltage = 0.04;
		break;
	case 1:
		max_shunt_voltage = 0.08;
		break;
	case 2:
		max_shunt_voltage = 0.16;
		break;
	case 3:
		max_shunt_voltage = 0.32;
		break;
	default:
		break;
	}
	return (double)(max_shunt_voltage/R_SHUNT);
}

/**
  * @retval HAL status
  */

double ina219_calculate_and_write_calibration_value(ina219_config ina219)
{
	double Current_LSB = ina219_calculate_max_expected_current(ina219) / pow(2,15);

	double Cal=0;

	Cal = trunc(FIXED_VALUE / (Current_LSB*R_SHUNT));

	ina219_write(INA219_CALIBRATION_REG_ADDR, Cal);

	return (double)Cal;
}

/**
  * @retval shunt voltage value mV
  */
double ina219_read_shunt_voltage(void)
{
	return (((ina219_read(INA219_SHUNT_VOLTAGE_REG_ADDR) * pow(10,-5)))*1000);
}

/**
  * @retval bus voltage value V
  */
double ina219_read_bus_voltage(void)
{
	return ((ina219_read(INA219_BUS_VOLTAGE_REG_ADDR) >> 3) * 4 * pow(10,-3));
}

/**
  * @retval read current mA
  */
double ina219_read_shunt_and_bus_current(ina219_config ina219)
{
	double Current_LSB = ina219_calculate_max_expected_current(ina219) / pow(2,15);
	return(1000*(((((ina219_read(INA219_SHUNT_VOLTAGE_REG_ADDR) * ina219_calculate_and_write_calibration_value(ina219)) / 4096)) * Current_LSB)));
}

/**
  * @retval read power mW
  */
double ina219_read_bus_power(ina219_config ina219)
{
	double Current_LSB = ina219_calculate_max_expected_current(ina219) / pow(2,15);
	double Power_LSB = 20 * Current_LSB;
	return (100*(Power_LSB*(ina219_read(INA219_CURRENT_REG_ADDR) * (ina219_read(INA219_BUS_VOLTAGE_REG_ADDR))/5000)));
}

/**
  * @brief Median Filter
  */
double ina219_moving_average_filter(double* array,uint64_t array_length)
{
	  double sum=0;
	  for(uint32_t count=0;count<array_length;count++)
	  {
		  sum += *(array+count);
	  }
	  return (sum/array_length);
}

/**
  * @brief System Correction
  */
static void error_signal(void)
{
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
}


/**
  * @brief System Correction
  */
static void complete_signal(void)
{
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
}

















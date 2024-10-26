/**
  ******************************************************************************
  * @file    ina219.h
  * @author  Kubilay Közleme
  * @brief   INA219 current measurement sensor driver library
  * @version V1
  * @first change time 08/06/24
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

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "main.h"

/**********************************************
 *  		  ina219 fixed values			  *
 **********************************************/
#define R_SHUNT		(0.1)
#define FIXED_VALUE (0.04096)

/**********************************************
 *  	ina219 median filter parameters		  *
 **********************************************/
#define MEDIAN_FILTER_N_VALUE	(100)
#define __array_length(buffer) 	(sizeof(buffer)/sizeof(buffer[0]))

/**********************************************
 *  ina219 slave device write and read address*
 **********************************************/
#define INA219_DEVICE_ADDR		(0x40U)
#define INA219_WRITE_ADDR		(INA219_DEVICE_ADDR << 1U)
#define INA219_READ_ADDR		((INA219_DEVICE_ADDR << 1U) | (0x1U))

/**********************************
 *  ina219 register map definition*
 **********************************/
#define INA219_CONFIG_REG_ADDR					(0x00U)		/*!< All-register reset|settings for bus voltage range|PGA Gain|ADC esolution/averaging 		 >!*/
#define INA219_SHUNT_VOLTAGE_REG_ADDR			(0x01U)		/*!< Shunt voltage measurement data 															 >!*/
#define INA219_BUS_VOLTAGE_REG_ADDR				(0x02U)		/*!< Bus voltage measurement data 																 >!*/
#define INA219_POWER_REG_ADDR					(0x03U)		/*!< Power measurement data 																	 >!*/
#define INA219_CURRENT_REG_ADDR					(0x04U)		/*!< Contains the value of the current flowing through the shunt resistor 						 >!*/
#define INA219_CALIBRATION_REG_ADDR				(0x05U)		/*!< Sets full-scale range and LSB of current and power measurements.Overall system calibration. >!*/

/***********************************
 *  ina219 register bit description*
 ***********************************/
/*Configuration Register [R/W]*/
//Reset Bit
#define INA219_RST_Pos		(15U)
#define INA219_RST_Msk		(0x1U << INA219_RST_Pos)
#define INA219_RST			INA219_RST_Msk

/***********************************
 *  ina219_config define parameters*
 ***********************************/

/**
 * @DEFINE_BUS_VOLTAGE_RANGE
 * BRNG (Bus Voltage Range)
 */
#define INA219_BUS_VOLTAGE_RANGE_16V		(0U)
#define INA219_BUS_VOLTAGE_RANGE_32V		(1U)


/*
 * @DEFINE_PGA
 * PGA (Shunt Voltage Only)
 */
#define INA219_GAIN_1		(0U)
#define INA219_GAIN_2		(1U)
#define INA219_GAIN_4		(2U)
#define INA219_GAIN_8		(3U)

/**
 * @DEFINE_BUS_ADC_RESOLUTION
 * BADC (Bus ADC Resolution/Averaging) ADC Settings
 */
#define INA219_BUS_ADC_MODE_9BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_BUS_ADC_MODE_10BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_BUS_ADC_MODE_11BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_BUS_ADC_MODE_12BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_BUS_ADC_SAMPLE_2			((0x01U << 3U) | (0x00U << 2U) | (0x00U << 1U) | (0x01U << 0U) )
#define INA219_BUS_ADC_SAMPLE_4			((0x01U << 3U) | (0x00U << 2U) | (0x01U << 1U) | (0x00U << 0U) )
#define INA219_BUS_ADC_SAMPLE_8			((0x01U << 3U) | (0x00U << 2U) | (0x01U << 1U) | (0x01U << 0U) )
#define INA219_BUS_ADC_SAMPLE_16		((0x01U << 3U) | (0x01U << 2U) | (0x00U << 1U) | (0x00U << 0U) )
#define INA219_BUS_ADC_SAMPLE_32		((0x01U << 3U) | (0x01U << 2U) | (0x00U << 1U) | (0x01U << 0U) )
#define INA219_BUS_ADC_SAMPLE_64		((0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | (0x00U << 0U) )
#define INA219_BUS_ADC_SAMPLE_128		((0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | (0x01U << 0U) )

/**
 * @DEFINE_SHUNT_ADC_RESOLUTION
 * SADC (Shunt ADC Resolution/Averaging)  ADC Settings
 */
#define INA219_SHUNT_ADC_MODE_9BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_MODE_10BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_MODE_11BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_MODE_12BIT		((0x00U << 3U) | (0x00U << 2U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_2		((0x01U << 3U) | (0x00U << 2U) | (0x00U << 1U) | (0x01U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_4		((0x01U << 3U) | (0x00U << 2U) | (0x01U << 1U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_8		((0x01U << 3U) | (0x00U << 2U) | (0x01U << 1U) | (0x01U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_16		((0x01U << 3U) | (0x01U << 2U) | (0x00U << 1U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_32		((0x01U << 3U) | (0x01U << 2U) | (0x00U << 1U) | (0x01U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_64		((0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | (0x00U << 0U) )
#define INA219_SHUNT_ADC_SAMPLE_128		((0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | (0x01U << 0U) )

/**
 * @DEFINE_OPERATING_MODE
 * MODE (Operating Mode)
 */
#define INA219_MODE_POWER_DOWN					((0x00U << 2U) | (0x00U << 1U) | (0x00U << 0U) )
#define INA219_MODE_SHUNT_VOLTAGE_TRIG			((0x00U << 2U) | (0x00U << 1U) | (0x01U << 0U) )
#define INA219_MODE_BUS_VOLTAGE_TRIG			((0x00U << 2U) | (0x01U << 1U) | (0x00U << 0U) )
#define INA219_MODE_SHUNT_AND_BUS_TRIG			((0x00U << 2U) | (0x01U << 1U) | (0x01U << 0U) )
#define INA219_MODE_ADC_OFF						((0x01U << 2U) | (0x00U << 1U) | (0x00U << 0U) )
#define INA219_MODE_SHUNT_VOLTAGE_CONTINOUS		((0x01U << 2U) | (0x00U << 1U) | (0x01U << 0U) )
#define INA219_MODE_BUS_VOLTAGE_CONTINOUS		((0x01U << 2U) | (0x01U << 1U) | (0x00U << 0U) )
#define INA219_MODE_SHUNT_AND_BUS_CONTINOUS		((0x01U << 2U) | (0x01U << 1U) | (0x01U << 0U) )


/**********************************
 *  ina219 config parameter struct*
 **********************************/
typedef struct
{
	uint8_t bus_voltage_range; 			/*!< ref @DEFINE_BUS_VOLTAGE_RANGE 	 	>!*/
	uint8_t pga;						/*!< ref @DEFINE_PGA				 	>!*/
	uint8_t bus_adc_resolution;			/*!< ref @DEFINE_BUS_ADC_RESOLUTION	 	>!*/
	uint8_t shunt_adc_resolution;		/*!< ref @DEFINE_SHUNT_ADC_RESOLUTION	>!*/
	uint8_t operating_mode;				/*!< ref @DEFINE_OPERATING_MODE         >!*/

}ina219_config;

/**********************************
 *  ina219 moving filter struct	  *
 **********************************/
struct ina219_moving_filter
{
	double bus_voltage[MEDIAN_FILTER_N_VALUE];
	double shunt_voltage[MEDIAN_FILTER_N_VALUE];
	double current[MEDIAN_FILTER_N_VALUE];
	double power[MEDIAN_FILTER_N_VALUE];
};

/**
 *  ina219 apis
 */
HAL_StatusTypeDef ina219_selftest(void);
void ina219_power_on_reset(void);

HAL_StatusTypeDef ina219_init(ina219_config);

double ina219_read_shunt_voltage(void);
double ina219_calculate_and_write_calibration_value(ina219_config);
double ina219_read_bus_voltage(void);
double ina219_read_shunt_and_bus_current(ina219_config);
double ina219_read_bus_power(ina219_config);

double ina219_moving_average_filter(double*,uint64_t);

#endif /* INC_INA219_H_ */









































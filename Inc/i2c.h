/**
  ******************************************************************************
  * File Name          : I2C.h
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define I2C_MAX_BYTES_TO_READ 4
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
// x: uint8_t i2c_master_read_byte(uint8_t slave_address, uint8_t register_address);
uint8_t* i2c_master_read_multi(uint8_t*buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag);
uint8_t i2c_master_read_single(uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag);

void i2c_master_write(uint8_t *buff, uint8_t len, uint8_t register_addr, uint8_t slave_addr, uint8_t read_flag);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

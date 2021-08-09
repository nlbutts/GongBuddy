/**
 ******************************************************************************
 * @file    custom_mems_conf.h
 * @author  MEMS Application Team
 * @brief   This file contains definitions of the MEMS components bus interfaces for custom boards
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_MEMS_CONF_H__
#define __CUSTOM_MEMS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "custom_bus.h"
#include "custom_errno.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define USE_CUSTOM_MOTION_SENSOR_LSM6DSL_0        1U

#define CUSTOM_LSM6DSL_0_SPI_Init BSP_SPI3_Init
#define CUSTOM_LSM6DSL_0_SPI_DeInit BSP_SPI3_DeInit
#define CUSTOM_LSM6DSL_0_SPI_Send BSP_SPI3_Send
#define CUSTOM_LSM6DSL_0_SPI_Recv BSP_SPI3_Recv

#define CUSTOM_LSM6DSL_0_CS_PORT GPIOB
#define CUSTOM_LSM6DSL_0_CS_PIN GPIO_PIN_12

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_MEMS_CONF_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
 * @file sensor_io.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief IO sub-system for the GongBuddy
 * @version 0.1
 * @date 2021-07-31
 *
 * @copyright Copyright (c) 2021
 *
 *
 */
#ifndef SENSOR_IO_H_
#define SENSOR_IO_H_

#include <stdint.h>
#include "main.h"

#define USE_MOTION_SENSOR_LSM6DSM_0        1U
#define USE_MOTION_SENSOR_LSM303AGR_ACC_0  0U
#define USE_MOTION_SENSOR_LSM303AGR_MAG_0  1U
#define USE_ENV_SENSOR_HTS221_0            0U
#define USE_ENV_SENSOR_LPS22HB_0           1U

#define BSP_LSM6DSM_INT2_GPIO_PORT           GPIOA
#define BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE()   __GPIOA_CLK_ENABLE()
#define BSP_LSM6DSM_INT2_GPIO_CLK_DISABLE()  __GPIOA_CLK_DISABLE()
#define BSP_LSM6DSM_INT2                 GPIO_PIN_2
#define BSP_LSM6DSM_INT2_EXTI_IRQn           EXTI2_IRQn

#define BSP_LSM6DSM_CS_PORT GPIOB
#define BSP_LSM6DSM_CS_PIN GPIO_PIN_12
#define BSP_LSM6DSM_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_M_CS_PORT GPIOB
#define BSP_LSM303AGR_M_CS_PIN GPIO_PIN_1
#define BSP_LSM303AGR_M_CS_GPIO_CLK_ENABLE()  __GPIOB_CLK_ENABLE()

#define BSP_LSM303AGR_X_CS_PORT GPIOC
#define BSP_LSM303AGR_X_CS_PIN GPIO_PIN_4
#define BSP_LSM303AGR_X_CS_GPIO_CLK_ENABLE()  __GPIOC_CLK_ENABLE()

#define BSP_LPS22HB_CS_PORT GPIOA
#define BSP_LPS22HB_CS_PIN GPIO_PIN_3
#define BSP_LPS22HB_CS_GPIO_CLK_ENABLE()  __GPIOA_CLK_ENABLE()

#define LORA_CS_PORT GPIOG
#define LORA_CS_PIN GPIO_PIN_12
#define LORA_CS_GPIO_CLK_ENABLE()  __GPIOG_CLK_ENABLE()

#define RADIO_NSS_PORT GPIOG
#define RADIO_NSS_PIN GPIO_PIN_12

/* Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10


/*
These are functions that are defined in lsm6dsl.h header
*/
int32_t sensor_Init_Func(void);
int32_t sensor_DeInit_Func(void);
int32_t sensor_GetTick_Func(void);
int32_t sensor_WriteReg_Func(uint16_t address, uint16_t register, uint8_t * pData, uint16_t len);
int32_t sensor_ReadReg_Func(uint16_t address, uint16_t register, uint8_t * pData, uint16_t len);

#endif /* SENSOR_IO_H_ */

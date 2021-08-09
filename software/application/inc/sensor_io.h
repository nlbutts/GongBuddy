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

/*
These are functions that are defined in lsm6dsl.h header
*/
int32_t sensor_Init_Func(void);
int32_t sensor_DeInit_Func(void);
int32_t sensor_GetTick_Func(void);
int32_t sensor_WriteReg_Func(uint16_t address, uint16_t register, uint8_t * pData, uint16_t len);
int32_t sensor_ReadReg_Func(uint16_t address, uint16_t register, uint8_t * pData, uint16_t len);

#endif /* SENSOR_IO_H_ */

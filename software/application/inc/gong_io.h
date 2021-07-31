/**
 * @file gong_io.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief IO sub-system for the GongBuddy
 * @version 0.1
 * @date 2021-07-30
 *
 * @copyright Copyright (c) 2021
 *
 *
 */
#ifndef GONG_IO_H_
#define GONG_IO_H_

#include <stdint.h>
#include "main.h"

/**
 * @brief Initialize the gong io expander
 *
 * @param spi a pointer to the SPI interface
 */
void gong_io_init(SPI_HandleTypeDef * spi);

/**
 * @brief Set the LED high/low
 *
 * @param high non-zero to enable, zero to set low
 */
void gong_io_set_led(int high);

/**
 * @brief Set the SD CS high/low
 *
 * @param high non-zero to enable, zero to set low
 */
void gong_io_set_sd_cs(int high);

/**
 * @brief Set the LORA Reset high/low
 *
 * @param high non-zero to enable, zero to set low
 */
void gong_io_set_lora_reset(int high);

/**
 * @brief Set the Lora CS high/low
 *
 * @param high non-zero to enable, zero to set low
 */
void gong_io_set_lora_cs(int high);

/**
 * @brief Read the Power Good (PG) from the regulator
 *
 * @return int state of the pin (0 or 1)
 */
int gong_io_read_pg();

/**
 * @brief Read the DIO0 pin
 *
 * @return int state of the pin (0 or 1)
 */
int gong_io_read_dio0();

/**
 * @brief Read the DIO1 pin
 *
 * @return int state of the pin (0 or 1)
 */
int gong_io_read_dio1();

/**
 * @brief Read the button
 *
 * @return int state of the pin (0 or 1)
 */
int gong_io_read_button();

#endif /* GONG_IO_H_ */

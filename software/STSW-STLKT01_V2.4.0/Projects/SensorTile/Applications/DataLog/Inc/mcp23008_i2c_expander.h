#pragma once

#include <stdint.h>

struct _mcp23008_i2c_expander_data
{
    uint8_t deviceAddress;
    uint8_t inputPorts; // Bit field, positive values are inputs, otherwise outputs
};

typedef struct _mcp23008_i2c_expander_data i2c_expander_data;

/*! @brief Initialize the I2C expander
 *  @param cfg the configuration struct
 *
 *  @return errno
 */
int i2c_expander_init(i2c_expander_data *cfg);

/*! @brief Generically write to the GPIO register
 *  @param data the parameter
 *
 *  @return errno
 */
int i2c_expander_write(uint8_t data);

/*! @brief Reads the GPIO register
 *
 *  @return errno
 */
int i2c_expander_read();

/*! @brief Sets or clear GPIO
 *  @param value 0 off 1 on
 *
 *  @return errno
 */
int i2c_big_led(uint8_t value);

/*! @brief Sets or clear GPIO
 *  @param value 0 off 1 on
 *
 *  @return errno
 */
int i2c_lora_reset(uint8_t value);

/*! @brief Sets or clear GPIO
 *  @param value 0 off 1 on
 *
 *  @return errno
 */
int i2c_sd_cs(uint8_t value);

/*! @brief Reads GPIO
 *  @return int errno if error otherwise value
 *
 *  @return errno
 */
int i2c_get_pg();

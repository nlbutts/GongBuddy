/**
 * @file mcp23S08.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief This file is a simple driver for the MCP23S08 SPI IO expanders.
 * @version 1.0
 * @date 2021-07-30
 *
 * @copyright Copyright (c) 2021
 *
 *
 * TODO: Implement interrupt support
 */

*/
#ifndef MCP23S08_H_
#define MCP23S08_H_

#include <stdint.h>

#define MCP23S08_REG_IODIR              0
#define MCP23S08_REG_IPOL               1
#define MCP23S08_REG_GPINTEN            2
#define MCP23S08_REG_DEFVAL             3
#define MCP23S08_REG_INTCON             4
#define MCP23S08_REG_GPPU               5
#define MCP23S08_REG_INTF               6
#define MCP23S08_REG_INTCAP             8
#define MCP23S08_REG_GPIO               9
#define MCP23S08_REG_OLAT               10

#define MCP23S08_WRITE                  0x40
#define MCP23S08_READ                   0x41

#define MCP23S08_IOCON_SEQ_OP_MODE_DIS  1 << 5
#define MCP23S08_IOCON_SLEW_RATE_DIS    1 << 4
#define MCP23S08_IOCON_HW_ADDR_EN       1 << 3
#define MCP23S08_IOCON_OPEN_DRAIN       1 << 2
#define MCP23S08_IOCON_INT_POL_HIGH     1 << 1

typedef int(*write_spi_fn)(uint8_t * data, uint8_t len);
typedef int(*tx_rx_spi_fn)(uint8_t * txdata, uint8_t * rxdata, uint8_t len);

struct mcp23s08_cfg
{
    uint8_t iodir;      // 1 is an input 0 is an output
    uint8_t iopol;      // 1 = invert input bit
    uint8_t gpinten;    // 1 enable GPIO input interrupt on change
    uint8_t defval;     // def value, will trigger interrupt if different from this value
    uint8_t intcon;     // 0 = interrupt on change, 1 = interrupt on defval
    uint8_t iocon;      // See flags above
    uint8_t gppu;       // GPIO pull up resistor 1 = enabled
    write_spi_fn write_spi; // Write SPI fn
    tx_rx_spi_fn tx_rx_spi; // tx/rx transfer
};

/**
 * @brief Initialize the mcp23s08 device
 *
 * @param cfg configuration structure
 * @return int zero if good, errno if bad
 */
int mcp23s08_init(struct mcp23s08_cfg *cfg);

/**
 * @brief Sets the GPIO pin
 *
 * @param pin
 * @return int zero if good, errno if bad
 */
int mcp23s08_set_pin(uint8_t gpio);

/**
 * @brief Clear the GPIO pin
 *
 * @param pin
 * @return int zero if good, errno if bad
 */
int mcp23s08_clear_pin(uint8_t gpio);

/**
 * @brief Toggle the GPIO pin
 *
 * @param pin
 * @return int zero if good, errno if bad
 */
int mcp23s08_toggle_pin(uint8_t gpio);

/**
 * @brief Read the interrupt pin that generated the interrupt
 *
 * @param pin
 * @return int zero if good, errno if bad
 */
int mcp23s08_read_interrupt_captured();

/**
 * @brief Read the interrupt flags
 *
 * @param pin
 * @return int zero if good, errno if bad
 */
int mcp23s08_read_interrupt_flag();

/**
 * @brief Read the interrupt flags
 *
 * @return int negative if failed read, otherwise value
 */
int mcp23s08_read_inputs();

#endif /* MCP23S08_H_ */

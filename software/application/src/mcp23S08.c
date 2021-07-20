#include <stdint.h>
#include "mcp23S08.h"

struct mcp23s08_cfg _cfg;

int mcp23s08_init(struct mcp23s08_cfg *cfg)
{
    _cfg = *cfg;

    uint8_t data[3];
    data[0] = 0x40;
    data[1] = MCP23S08_REG_IODIR;
    data[2] = _cfg.iodir;
    _cfg.write_spi(data, 3);
    _cfg.write_spi(data, 3);

    return 0;
}

static int read_olat()
{
    uint8_t txdata[3];
    uint8_t rxdata[3];
    txdata[0] = MCP23S08_READ;
    txdata[1] = MCP23S08_REG_OLAT;
    if (_cfg.tx_rx_spi(txdata, rxdata, 3) != 0)
        return -1;
    return rxdata[2];
}

int mcp23s08_set_pin(uint8_t gpio)
{
    uint8_t data[3];
    int olat = read_olat();
    if (olat >= 0)
    {
        data[0] = 0x40;
        data[1] = MCP23S08_REG_GPIO;
        data[2] = olat | gpio;
        if (_cfg.write_spi(data, 3) == 0)
        {
            return 0;
        }
    }
    return -1;
}

int mcp23s08_clear_pin(uint8_t gpio)
{
    uint8_t data[3];
    int olat = read_olat();
    if (olat >= 0)
    {
        data[0] = 0x40;
        data[1] = MCP23S08_REG_GPIO;
        data[2] = olat & ~gpio;
        if (_cfg.write_spi(data, 3) == 0)
        {
            return 0;
        }
    }
    return -1;

}

int mcp23s08_toggle_pin(uint8_t gpio)
{
    uint8_t data[3];
    int olat = read_olat();
    if (olat >= 0)
    {
        data[0] = 0x40;
        data[1] = MCP23S08_REG_GPIO;
        data[2] = olat ^ gpio;
        if (_cfg.write_spi(data, 3) == 0)
        {
            return 0;
        }
    }
    return -1;
}

int mcp23s08_read_interrupt_captured()
{

}

int mcp23s08_read_interrupt_flag()
{

}

int mcp23s08_read_inputs()
{
    uint8_t txdata[3];
    uint8_t rxdata[3];
    txdata[0] = MCP23S08_READ;
    txdata[1] = MCP23S08_REG_GPIO;
    if (_cfg.tx_rx_spi(txdata, rxdata, 3) != 0)
        return -1;
    return rxdata[2];
}

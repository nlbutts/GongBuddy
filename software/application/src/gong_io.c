#include <stdint.h>
#include "mcp23S08.h"
#include "main.h"

#define IO_PG           0x01
#define IO_DIO0         0x02
#define IO_DIO1         0x04
#define IO_LED          0x08
#define IO_BUTTON       0x10
#define IO_SD_CS        0x20
#define IO_LORA_RESET   0x40
#define IO_LORA_CS      0x80

static void   *_spi;

static int write_spi(uint8_t * data, uint8_t len)
{
    HAL_GPIO_WritePin(IO_CS_GPIO_Port, IO_CS_Pin, 0);
    HAL_SPI_Transmit(_spi, data, len, 1000);
    HAL_GPIO_WritePin(IO_CS_GPIO_Port, IO_CS_Pin, 1);
    return 0;
}

static int tx_rx_spi(uint8_t * txdata, uint8_t * rxdata, uint8_t len)
{
    HAL_GPIO_WritePin(IO_CS_GPIO_Port, IO_CS_Pin, 0);
    HAL_SPI_TransmitReceive(_spi, txdata, rxdata, len, 100);
    HAL_GPIO_WritePin(IO_CS_GPIO_Port, IO_CS_Pin, 1);
    return 0;
}

void gong_io_init(void * spi)
{
    struct mcp23s08_cfg cfg = {0};
    _spi = spi;
    cfg.iodir = 0x17;
    cfg.write_spi = &write_spi;
    cfg.tx_rx_spi = &tx_rx_spi;
    mcp23s08_init(&cfg);
    mcp23s08_set_pin(0xFF);
}

void gong_io_set_led(int high)
{
    if (high)
        mcp23s08_set_pin(IO_LED);
    else
        mcp23s08_clear_pin(IO_LED);
}

void gong_io_set_sd_cs(int high)
{
    if (high)
        mcp23s08_set_pin(IO_SD_CS);
    else
        mcp23s08_clear_pin(IO_SD_CS);
}

void gong_io_set_lora_reset(int high)
{
    if (high)
        mcp23s08_set_pin(IO_LORA_RESET);
    else
        mcp23s08_clear_pin(IO_LORA_RESET);
}

void gong_io_set_lora_cs(int high)
{
    if (high)
        mcp23s08_set_pin(IO_LORA_CS);
    else
        mcp23s08_clear_pin(IO_LORA_CS);
}

int gong_io_read_pg()
{
    int value;
    int pins = mcp23s08_read_inputs();
    if (pins >= 0)
        value = pins & IO_PG;
    else
        value = pins;
    return value;
}

int gong_io_read_dio0()
{
    int value;
    int pins = mcp23s08_read_inputs();
    if (pins >= 0)
        value = pins & IO_DIO0;
    else
        value = pins;
    return value;
}

int gong_io_read_dio1()
{
    int value;
    int pins = mcp23s08_read_inputs();
    if (pins >= 0)
        value = pins & IO_DIO1;
    else
        value = pins;
    return value;
}

int gong_io_read_button()
{
    int value;
    int pins = mcp23s08_read_inputs();
    if (pins >= 0)
        value = pins & IO_BUTTON;
    else
        value = pins;
    return value;
}

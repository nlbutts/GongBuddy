#include "mcp23008_i2c_expander.h"
#include <cube_hal.h>
#include <errno.h>
#include <cmsis_os.h>

/*
* This is the driver for the MCP23008 I2C expander
*/

enum I2C_REGS
{
    IODIR,      // 0x0
    IPOL,       // 0x1
    GPINTEN,    // 0x2
    DEFVAL,     // 0x3
    INTCON,     // 0x4
    IOCON,      // 0x5
    GPPU,       // 0x6
    INTF,       // 0x7
    INTCAP,     // 0x8
    GPIO,       // 0x9
    OLAT        // 0xA
};

static i2c_expander_data _cfg;
static I2C_HandleTypeDef hi2c3;
osMutexDef(lock);
osMutexId _lock_id;
uint8_t _currentValue = 0;

int i2c_expander_init(i2c_expander_data *cfg)
{
    int rv = 0;
    _cfg = *cfg;

    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable VddIO2 for GPIOG  */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C3_CLK_ENABLE();

    /*** Configure the GPIOs ***/
    GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Get the APB1 bus speed
    volatile uint32_t apb1clk = HAL_RCC_GetPCLK1Freq();

    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = 0x212139D8;
    //hi2c3.Init.Timing = 0x30213030;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        rv = -EBUSY;
    }
    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        rv = -EBUSY;
    }
    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
    {
        rv = -EBUSY;
    }

    // Set the GPIO
    uint8_t buf[2] = {IODIR, _cfg.inputPorts};
    if (HAL_OK != HAL_I2C_Master_Transmit(&hi2c3, _cfg.deviceAddress, buf, 2, 1000))
    {
        rv = -EBUSY;
    }

    _lock_id = osMutexCreate(osMutex(lock));
    if (_lock_id == NULL)
    {
        rv = -EBUSY;
    }

    return rv;
}

int i2c_expander_write(uint8_t data)
{
    int rv = 0;
    uint8_t buf[2] = {GPIO, data};
    osMutexWait(_lock_id, osWaitForever);
    if (HAL_OK != HAL_I2C_Master_Transmit(&hi2c3, _cfg.deviceAddress, buf, 2, 1000))
    {
        rv = -EBUSY;
    }
    osMutexRelease(_lock_id);
    return rv;
}

static int i2c_expander_reg(uint8_t reg)
{
    int rv = 0;
    uint8_t buf[2] = {reg, 0};
    osMutexWait(_lock_id, osWaitForever);
    if (HAL_OK != HAL_I2C_Master_Transmit(&hi2c3, _cfg.deviceAddress, buf, 1, 1000))
    {
        rv = -EBUSY;
    }
    if (HAL_OK != HAL_I2C_Master_Receive(&hi2c3, _cfg.deviceAddress, buf, 1, 1000))
    {
        rv = -EBUSY;
    }
    else
    {
        rv = buf[0];
    }

    osMutexRelease(_lock_id);
    return rv;
}

int i2c_expander_read()
{
    return i2c_expander_reg(GPIO);
}

int setIO(uint8_t value, uint8_t bit)
{
    //int rv = i2c_expander_read();
    //if (rv >= 0)
    {
        uint8_t mask = 1 << bit;
        _currentValue &= ~mask;
        _currentValue |= value << bit;
        i2c_expander_write(_currentValue);
    }
    return 0;
}

int i2c_big_led(uint8_t value)
{
    return setIO(value, 0);
}

int i2c_debug1(uint8_t value)
{
    return setIO(value, 1);
}

int i2c_debug2(uint8_t value)
{
    return setIO(value, 2);
}

int i2c_lora_reset(uint8_t value)
{
    return setIO(value, 5);
}

int i2c_sd_cs(uint8_t value)
{
    return setIO(value, 7);
}

int i2c_get_pg()
{
    int rv = i2c_expander_read();
    if (rv >= 0)
    {
        rv = (rv >> 6) & 1;
    }
    return rv;
}
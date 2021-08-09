#include "sensor_io.h"
#include "gong_io.h"

int32_t sensor_Init_Func(void)
{
    return 0;
}

int32_t sensor_DeInit_Func(void)
{
    return 0;
}

int32_t sensor_GetTick_Func(void)
{
    return 0;
}

int32_t sensor_WriteReg_Func(uint16_t address, uint16_t reg, uint8_t * pData, uint16_t len)
{
    int32_t rv = 0;
    if (len == 1)
    {
        HAL_StatusTypeDef status;
        uint8_t buf[2] = {reg, pData[0]};
        HAL_GPIO_WritePin(AG_CS_GPIO_Port, AG_CS_Pin, 0);
        status = HAL_SPI_Transmit(&hspi2, buf, 2, 1000);
        HAL_GPIO_WritePin(AG_CS_GPIO_Port, AG_CS_Pin, 1);
        if (status != HAL_OK)
        {
            rv = -1;
        }
    }
    return rv;
}

int32_t sensor_ReadReg_Func(uint16_t address, uint16_t reg, uint8_t * pData, uint16_t len)
{
    int32_t rv = 0;
    if (len == 1)
    {
        HAL_StatusTypeDef status;
        uint8_t txbuf[2] = {reg | 0x80, 0};
        HAL_GPIO_WritePin(AG_CS_GPIO_Port, AG_CS_Pin, 0);
        status = HAL_SPI_Transmit(&hspi2, txbuf, 1, 1000);
        status = HAL_SPI_Receive(&hspi2, pData, 1, 1000);
        //status = HAL_SPI_TransmitReceive(&hspi2, txbuf, rxbuf, 2, 1000);
        HAL_GPIO_WritePin(AG_CS_GPIO_Port, AG_CS_Pin, 1);
        if (status != HAL_OK)
        {
            rv = -1;
        }
    }
    return rv;
}

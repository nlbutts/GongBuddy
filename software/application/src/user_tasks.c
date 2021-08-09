#include "user_tasks.h"
#include "cmsis_os.h"
#include "debug_print.h"
#include "lsm6dsm.h"
#include "sensor_io.h"
#include "gong_io.h"

osThreadId_t data_producer_handle;
const osThreadAttr_t data_producer_attributes = {
  .name = "data_producer_task",
  .stack_size = 128 * 10,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t business_logic_handle;
const osThreadAttr_t business_logic_attributes = {
  .name = "business_logic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t sd_data_logger_handle;
const osThreadAttr_t sd_data_logger_attributes = {
  .name = "sd_logger",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

static int32_t LSM6DSM_Sensor_IO_ITConfig( void )
{
    /* At the moment this feature is only implemented for LSM6DSM */
    GPIO_InitTypeDef GPIO_InitStructureInt2;

    /* Enable INT2 GPIO clock */
    BSP_LSM6DSM_INT2_GPIO_CLK_ENABLE();

    /* Configure GPIO PINs to detect Interrupts */
    GPIO_InitStructureInt2.Pin = BSP_LSM6DSM_INT2;
    GPIO_InitStructureInt2.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructureInt2.Speed = GPIO_SPEED_FAST;
    GPIO_InitStructureInt2.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(BSP_LSM6DSM_INT2_GPIO_PORT, &GPIO_InitStructureInt2);

    /* Enable and set EXTI Interrupt priority */
    HAL_NVIC_SetPriority(BSP_LSM6DSM_INT2_EXTI_IRQn, 0x08, 0x00);
    HAL_NVIC_EnableIRQ(BSP_LSM6DSM_INT2_EXTI_IRQn);

    return BSP_ERROR_NONE;
}


static void data_producer_task(void *arguments)
{
    LSM6DSM_IO_t io;
    io.Init = sensor_Init_Func;
    io.DeInit = sensor_DeInit_Func;
    io.BusType = 2;
    io.Address = 0;
    io.WriteReg = sensor_WriteReg_Func;
    io.ReadReg = sensor_ReadReg_Func;
    io.GetTick = sensor_GetTick_Func;

    LSM6DSM_Sensor_IO_ITConfig();

    LSM6DSM_Object_t sensor = {0};
    LSM6DSM_RegisterBusIO(&sensor, &io);
    uint8_t value;
    LSM6DSM_ReadID(&sensor, &value);
    LSM6DSM_ACC_Enable(&sensor);
    LSM6DSM_GYRO_Enable(&sensor);

    float sensitivity;
    float accel_odr;
    float gyro_odr;
    LSM6DSM_ACC_SetOutputDataRate(&sensor, 100.0f);
    LSM6DSM_ACC_GetOutputDataRate(&sensor, &accel_odr);

    LSM6DSM_GYRO_SetOutputDataRate(&sensor, 100.0f);
    LSM6DSM_GYRO_GetOutputDataRate(&sensor, &gyro_odr);
    DEBUG_PRINTF(0, "LSM6DS: %02x ACCEL ODR: %d GYRO ODR: %d\n", value, (int)accel_odr, (int)gyro_odr);

    int counter = 0;
    LSM6DSM_Axes_t accel;
    LSM6DSM_Axes_t gyro;

    for (;;)
    {
        osDelay(1000);
        LSM6DSM_ACC_GetAxes(&sensor, &accel);
        LSM6DSM_GYRO_GetAxes(&sensor, &gyro);
        DEBUG_PRINTF(0, "ACCEL: \t%d \t%d \t%d\n", accel.x, accel.y, accel.z);
        DEBUG_PRINTF(0, "GYRO: \t%d \t%d \t%d\n", gyro.x, gyro.y, gyro.z);
    }
}

static void business_logic_task(void *arguments)
{
    int counter = 0;
    for (;;)
    {
        osDelay(1000);
        //DEBUG_PRINTF(0, "business_logic_task: %d\n", counter++);
    }
}

static void sd_data_logger_task(void *arguments)
{
    int counter = 0;
    uint8_t state = 0;
    for (;;)
    {
        osDelay(1000);
        gong_io_set_led(state);
        state = ~state;
        //DEBUG_PRINTF(0, "sd_data_logger_task: %d\n", counter++);
    }
}

void tasks_init()
{
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    data_producer_handle = osThreadNew(data_producer_task, NULL, &data_producer_attributes);
    business_logic_handle = osThreadNew(business_logic_task, NULL, &business_logic_attributes);
    sd_data_logger_handle = osThreadNew(sd_data_logger_task, NULL, &sd_data_logger_attributes);
}

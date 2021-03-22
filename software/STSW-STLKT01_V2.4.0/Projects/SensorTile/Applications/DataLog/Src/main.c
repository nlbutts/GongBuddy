
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "datalog_application.h"
#include "FreeRTOS_CLI.h"
#include "pingpong.h"
#include "commandline.h"
#include "gb_messages.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "common_structs.h"
#include "utilities_conf.h"


#pragma GCC optimize("O0")
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define DATAQUEUE_SIZE     ((uint32_t)100)

#define DATALOG_CMD_STARTSTOP  (0x00000007)

typedef enum
{
  THREAD_1 = 0,
  THREAD_2,
  THREAD_3,
  THREAD_4,
  THREAD_5
} Thread_TypeDef;

/* Private variables ---------------------------------------------------------*/

osThreadId GetDataThreadId;
osThreadId WriteDataThreadId;
osThreadId ledThreadId;
osThreadId pingpongId;
osThreadId commandConsoleId;
osThreadId reprogrammingId;

osMessageQId dataQueue_id;
osMessageQDef(dataqueue, DATAQUEUE_SIZE, int);

osPoolId sensorPool_id;
osPoolDef(sensorPool, DATAQUEUE_SIZE, T_SensorsData);

osSemaphoreId readDataSem_id;
osSemaphoreDef(readDataSem);

osSemaphoreId ledBlinkSem_id;
osSemaphoreDef(ledBlinkSem);

osMessageQId loraQueue_id;
osMessageQDef(loraQueue, 1, int);

osPoolId loraPool_id;
osPoolDef(loraPool, 1, LoraData_t);

/* LoggingInterface = USB_Datalog  --> Send sensors data via USB */
/* LoggingInterface = SDCARD_Datalog  --> Save sensors data on SDCard (enable with double tap) */
LogInterface_TypeDef LoggingInterface = SDCARD_Datalog;

USBD_HandleTypeDef  USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
volatile uint8_t no_H_HTS221 = 0;
volatile uint8_t no_T_HTS221 = 0;

/* Private function prototypes -----------------------------------------------*/
static void GetData_Thread(void const *argument);
static void WriteData_Thread(void const *argument);
static void blinkLedThread(void const *argument);
static void sendLora(LoraMsg2 *inMsg, LoraMsg2 *outMsg);
static void setupReprogramming(void);
static void reprogrammingThread(void const *argument);

static void Error_Handler( void );
void dataTimer_Callback(void const *arg);
void dataTimerStart(void);
void dataTimerStop(void);

osTimerId sensorTimId;
osTimerDef(SensorTimer, dataTimer_Callback);

uint32_t  exec;

static int32_t LSM6DSM_Sensor_IO_ITConfig( void );

/* Private functions ---------------------------------------------------------*/
uint8_t detectImpact(T_SensorsData *sensorData, int threshold)
{
    float mag = (sensorData->acc.x * sensorData->acc.x) +
                (sensorData->acc.y * sensorData->acc.y) +
                (sensorData->acc.z * sensorData->acc.z);

    if (mag > (threshold*threshold))
    {
        return 1;
    }
  return 0;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    SET_BIT(RCC->CSR, 1 << 23);
    HAL_Init();

    /* Configure the System clock to 80 MHz */
    SystemClock_Config();

    /* Initialize LED */
    // BSP_LED_Init(LED1);
    // BSP_LED_Off(LED1);

    /* enable USB power on Pwrctrl CR2 register */
    HAL_PWREx_EnableVddUSB();
    HAL_PWREx_EnableVddIO2();

    /*** USB CDC Configuration ***/
    /* Init Device Library */
    USBD_Init(&USBD_Device, &VCP_Desc, 0);
    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
    /* Add Interface callbacks for AUDIO and CDC Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
    /* Start Device Process */
    USBD_Start(&USBD_Device);

    i2c_expander_data i2c_cfg;
    i2c_cfg.deviceAddress = 0x40; // Includes w/r bit
    i2c_cfg.inputPorts = 0x40;
    i2c_cfg.baudrate = 400000;
    i2c_expander_init(&i2c_cfg);
    i2c_lora_reset(1);
    i2c_lora_reset(0);
    i2c_big_led(1);
    i2c_big_led(0);

    osThreadDef(GetData_Thread,         GetData_Thread,       osPriorityAboveNormal,  0, configMINIMAL_STACK_SIZE*8);
    osThreadDef(WriteData_Thread,       WriteData_Thread,     osPriorityNormal,       0, configMINIMAL_STACK_SIZE*8);
    osThreadDef(blinkLedThread,         blinkLedThread,       osPriorityNormal,       0, configMINIMAL_STACK_SIZE);
    //osThreadDef(vCommandConsoleTask,    vCommandConsoleTask,  osPriorityNormal,       0, configMINIMAL_STACK_SIZE);

    GetDataThreadId     = osThreadCreate(osThread(GetData_Thread), NULL);
    WriteDataThreadId   = osThreadCreate(osThread(WriteData_Thread), NULL);
    ledThreadId         = osThreadCreate(osThread(blinkLedThread), NULL);
    //commandConsoleId    = osThreadCreate(osThread(vCommandConsoleTask), NULL);

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    for (;;);
}

/**
  * @brief  Get data raw from sensors to queue
  * @param  thread not used
  * @retval None
  */
static void GetData_Thread(void const *argument)
{
    (void) argument;
    T_SensorsData *mptr;

    sensorPool_id = osPoolCreate(osPool(sensorPool));
    dataQueue_id = osMessageCreate(osMessageQ(dataqueue), NULL);

    readDataSem_id = osSemaphoreCreate(osSemaphore(readDataSem), 1);
    osSemaphoreWait(readDataSem_id, osWaitForever);

    /* Initialize and Enable the available sensors */
    Sensor_IO_SPI_CS_Init_All();
    MX_X_CUBE_MEMS1_Init();

    /* COnfigure LSM6DSM Double Tap interrupt*/
    LSM6DSM_Sensor_IO_ITConfig();

    dataTimerStart();

    for (;;)
    {
        osSemaphoreWait(readDataSem_id, osWaitForever);
        /* Try to allocate a memory block and check if is not NULL */
        mptr = osPoolAlloc(sensorPool_id);
        if(mptr != NULL)
        {
            if(getSensorsData(mptr) == BSP_ERROR_NONE)
            {
                /* Push the new memory Block in the Data Queue */
                if(osMessagePut(dataQueue_id, (uint32_t)mptr, osWaitForever) != osOK)
                {
                    Error_Handler();
                }
            }
            else
            {
                Error_Handler();
            }
        }
        else
        {
            Error_Handler();
        }
    }
}

void packSensorSample(T_SensorsData * sample, uint8_t * buf)
{
    buf[0]  = (sample->acc.x >> 0)  & 0xFF;
    buf[1]  = (sample->acc.x >> 8)  & 0xFF;
    buf[2]  = (sample->acc.y >> 0)  & 0xFF;
    buf[3]  = (sample->acc.y >> 8)  & 0xFF;
    buf[4]  = (sample->acc.z >> 0)  & 0xFF;
    buf[5]  = (sample->acc.z >> 8)  & 0xFF;
    buf[6]  = (sample->gyro.x >> 0) & 0xFF;
    buf[7]  = (sample->gyro.x >> 8) & 0xFF;
    buf[8]  = (sample->gyro.y >> 0) & 0xFF;
    buf[9]  = (sample->gyro.y >> 8) & 0xFF;
    buf[10] = (sample->gyro.z >> 0) & 0xFF;
    buf[11] = (sample->gyro.z >> 8) & 0xFF;
}

#define SENSOR_CIR_BUF_SIZE 15
static T_SensorsData sensorBuffer[SENSOR_CIR_BUF_SIZE] = {0};
/**
  * @brief  Write data in the queue on file or streaming via USB
  * @param  argument not used
  * @retval None
  */
static void WriteData_Thread(void const *argument)
{
    (void) argument;
    osEvent evt;
    T_SensorsData *rptr;
    uint8_t impactDetected = 0;
    uint32_t sensorBufferIndex = 0;

    SD_IO_Init_LS();
    Sensor_init_lora_interfaces();
    LoRa_init();

    // Read the unique device ID
    uint32_t uuid = HAL_GetUIDw0();

    int threshold = 1500;
    int heartbeat_counter = 0;

    for (;;)
    {
        evt = osMessageGet(dataQueue_id, osWaitForever);  // wait for message
        i2c_debug2(1);
        heartbeat_counter++;
        rptr = evt.value.p;

        if (detectImpact(rptr, threshold) && (impactDetected == 0))
        {
            // Set a timer to roll the SD card recording
            impactDetected = 1;
            osSignalSet(ledThreadId, 0x10000);
        }

        if (impactDetected == 0)
        {
            // Copy sensor data into circular buffer
            sensorBuffer[0] = *rptr;
            sensorBufferIndex = 1;
        }
        else if (sensorBufferIndex < SENSOR_CIR_BUF_SIZE)
        {
            sensorBuffer[sensorBufferIndex++] = *rptr;
        }
        else if (sensorBufferIndex >= SENSOR_CIR_BUF_SIZE)
        {
            uint32_t dstDev;
            LoraMsg2 txMsg = LoraMsg2_init_default;
            LoraMsg2 rxMsg = LoraMsg2_init_default;
            sensorBufferIndex++;
            // Generate the protobuf and send it to the Lora Thread
            txMsg.status = Status_IMPACT;
            txMsg.identifier = uuid;
            txMsg.has_buildnum = true;
            txMsg.buildnum = 123;
            txMsg.has_pressure = true;
            txMsg.pressure = (rptr->pressure * 10);
            txMsg.has_temperature = true;
            txMsg.temperature = (rptr->temperature * 10);
            txMsg.has_threshold = true;
            txMsg.threshold = threshold;

            txMsg.has_imu = true;
            for (int i = 0; i < SENSOR_CIR_BUF_SIZE; i++)
            {
                packSensorSample(&sensorBuffer[i], &txMsg.imu.bytes[i*12]);
            }
            txMsg.imu.size = SENSOR_CIR_BUF_SIZE * 12;
            heartbeat_counter = 0;
            int reprogramming;
            sendLora(&txMsg, &rxMsg);

            impactDetected = 0;
        }

        if (heartbeat_counter > 500)
        {
            heartbeat_counter = 0;
            LoraMsg2 txMsg = LoraMsg2_init_default;
            LoraMsg2 rxMsg = LoraMsg2_init_default;
            txMsg.identifier = uuid;
            txMsg.status = Status_HEARTBEAT;
            txMsg.has_buildnum = true;
            txMsg.buildnum = 123;
            txMsg.has_pressure = true;
            txMsg.pressure = (rptr->pressure * 10);
            txMsg.has_temperature = true;
            txMsg.temperature = (rptr->temperature * 10);
            txMsg.has_threshold = true;
            txMsg.threshold = threshold;

            int reprogramming;
            sendLora(&txMsg, &rxMsg);

            if (rxMsg.identifier == uuid)
            {
                if (rxMsg.has_threshold && (rxMsg.threshold > 1500))
                {
                    threshold = rxMsg.threshold;
                }
                if (rxMsg.status == Status_REPROGRAMMING)
                {
                    setupReprogramming();
                }
                DBGPRINTF("Sent heartbeat, received t:%d s:%d\n", threshold, rxMsg.status);
            }

        }

        // unsigned int pressure = (rptr->pressure * 10);
        // unsigned int temperature = (rptr->temperature * 10);
        // size = snprintf(data_s,
        //                 SD_STR_SIZE,
        //                 "%u, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n",
        //             (unsigned int)rptr->ms_counter,
        //             (int)rptr->acc.x, (int)rptr->acc.y, (int)rptr->acc.z,
        //             (int)rptr->gyro.x, (int)rptr->gyro.y, (int)rptr->gyro.z,
        //             (int)rptr->mag.x, (int)rptr->mag.y, (int)rptr->mag.z,
        //             pressure, temperature, 0);

        osStatus status = osPoolFree(sensorPool_id, rptr);      // free memory allocated for message
        if (status < 0)
        {
            Error_Handler();
        }

        i2c_debug2(0);
    }
}

static void sendLora(LoraMsg2 *inMsg, LoraMsg2 *outMsg)
{
    uint8_t pb_data[250];

    pb_ostream_t stream = pb_ostream_from_buffer(pb_data, sizeof(pb_data));
    pb_encode(&stream, LoraMsg2_fields, inMsg);
    int retSize =
        LoRa_dataexchange(pb_data, stream.bytes_written, pb_data, sizeof(pb_data));
    pb_istream_t istrm = pb_istream_from_buffer(pb_data, retSize);
    pb_decode(&istrm, LoraMsg2_fields, outMsg);
}

static void setupReprogramming(void)
{
    osThreadDef(reprogrammingThread,         reprogrammingThread,       osPriorityAboveNormal,  0, configMINIMAL_STACK_SIZE*8);
    // Start the reprogramming code
    reprogrammingId = osThreadCreate(osThread(reprogrammingThread), NULL);
}

static uint64_t getBigInt(uint8_t * buf)
{
    uint64_t bigint = 0;
    for (int i = 0; i < 8; i++)
    {
        bigint |= ((uint64_t)buf[i]) << (8 * i);
    }
    return bigint;
}

static void reprogrammingThread(void const *argument)
{
    uint8_t pb_data[250];

    uint32_t uuid = HAL_GetUIDw0();
    uint32_t start = xTaskGetTickCount();

    osStatus status;

    DBGPRINTF("Entering programming mode\n");

    status = osThreadTerminate(GetDataThreadId);
    if (status != osOK)
    {
        DBGPRINTF("Failed to kill GetDataThread\n");
    }

    status = osThreadTerminate(WriteDataThreadId);
    if (status != osOK)
    {
        DBGPRINTF("Failed to kill WriteDataThread\n");
    }

    DBGPRINTF("Erasing flash\n");
    HAL_StatusTypeDef flashStatus;
    FLASH_EraseInitTypeDef eraseop;
    uint32_t pageError;
    eraseop.TypeErase  = FLASH_TYPEERASE_PAGES;
    eraseop.Banks      = FLASH_BANK_2;
    eraseop.Page       = 0;
    eraseop.NbPages    = 64;
    __HAL_FLASH_CLEAR_FLAG(0xFFFFFFFF);
    HAL_FLASH_Unlock();
    flashStatus = HAL_FLASHEx_Erase(&eraseop, &pageError);
    if (flashStatus != HAL_OK)
    {
        DBGPRINTF("Failed to erase flash\n");
    }

    uint32_t progAddress = 0;
    uint32_t rxAddress;
    int run = 1;

    while (run)
    {
        uint32_t stop = xTaskGetTickCount();
        if ((stop - start) > 10000)
        {
            DBGPRINTF("Programming timeout, rebooting\n");
            __NVIC_SystemReset();
        }
        osSignalSet(ledThreadId, 0x10000);

        LoraMsg2 msg = LoraMsg2_init_default;
        msg.identifier = uuid;
        msg.status = Status_REPROGRAMMING;
        msg.has_reprog = true;
        msg.reprog.address = progAddress;
        msg.reprog.flags = Reprogramming_Flags_CONTINUE;
        msg.reprog.data.size = 0;

        int reprogramming = 0;

        pb_ostream_t stream = pb_ostream_from_buffer(pb_data, sizeof(pb_data));
        pb_encode(&stream, LoraMsg2_fields, &msg);
        int retSize =
            LoRa_dataexchange(pb_data, stream.bytes_written, pb_data, sizeof(pb_data));
        LoraMsg2 inMsg = LoraMsg2_init_default;
        pb_istream_t istrm = pb_istream_from_buffer(pb_data, retSize);
        pb_decode(&istrm, LoraMsg2_fields, &inMsg);
        if (inMsg.has_reprog)
        {
            rxAddress = inMsg.reprog.address;
            if (rxAddress != progAddress)
            {
                DBGPRINTF("Error, required address and send address different: %08X %08X\n", progAddress, rxAddress);
            }
            else
            {
                DBGPRINTF("ADDR: %08X data[0]: %02X flags: %d\n", progAddress, inMsg.reprog.data.bytes[0], inMsg.reprog.flags);
                progAddress += inMsg.reprog.data.size;
                rxAddress += 0x8080000;
                for (int i = 0; i < inMsg.reprog.data.size; i += 8, rxAddress += 8)
                {
                    uint64_t bigint = getBigInt(&inMsg.reprog.data.bytes[i]);
                    __HAL_FLASH_CLEAR_FLAG(0xFFFFFFFF);
                    flashStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, rxAddress, bigint);
                    if (flashStatus != HAL_OK)
                    {
                        DBGPRINTF("Failed to program flash\n");
                    }
                    else
                    {
                        start = xTaskGetTickCount();
                    }

                }
            }

            if (inMsg.reprog.flags == Reprogramming_Flags_LAST_PACKET)
            {
                run = 0;
            }
        }
        osDelay(20);
    }

    DBGPRINTF("Data received, verifying CRC\n");
    DBGPRINTF("Decompressing and programming\n");
    DBGPRINTF("Done\n");
}

static void blinkLedThread(void const *argument)
{
    (void) argument;

    uint8_t value = 0;

    for (;;)
    {
        osSignalWait(0x10000, osWaitForever);

        for (int i = 0; i < 10; i++)
        {
            osDelay(250);
            value = !value;
            i2c_big_led(value);
        }
    }
}

void dataTimer_Callback(void const *arg)
{
    osSemaphoreRelease(readDataSem_id);
}


void dataTimerStart(void)
{
    osStatus  status;

    // Create periodic timer
    exec = 1;
    sensorTimId = osTimerCreate(osTimer(SensorTimer), osTimerPeriodic, &exec);
    if (sensorTimId)
    {
        status = osTimerStart (sensorTimId, DATA_PERIOD_MS);                // start timer
        if (status != osOK)
        {
            // Timer could not be started
        }
    }
}

void dataTimerStop(void)
{
  osTimerStop(sensorTimId);
}



/**
 * @brief  Configures sensor interrupts interface for LSM6DSM sensor.
 * @param  None
 * @retval BSP_ERROR_NONE in case of success
 * @retval COMPONENT_ERROR in case of failure
 */
int32_t LSM6DSM_Sensor_IO_ITConfig( void )
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



/**
* @brief  EXTI line detection callbacks
* @param  GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
    MEMSInterrupt=1;
    osSemaphoreRelease(readDataSem_id);
}

/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void Error_Handler( void )
{
    // volatile HeapStats_t stats;
    // vPortGetHeapStats(&stats);
    volatile int foo = 0;
    while (1)
    {
        foo++;
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, signed char *pcTaskName)
{
    while (1)
    {
        volatile TaskHandle_t localTask = task;
    }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
}
#endif

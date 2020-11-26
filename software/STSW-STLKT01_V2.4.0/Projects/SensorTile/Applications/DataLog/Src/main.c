
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "datalog_application.h"
#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define DATAQUEUE_SIZE     ((uint32_t)100)

#define DATALOG_CMD_STARTSTOP  (0x00000007)

typedef enum
{
  THREAD_1 = 0,
  THREAD_2,
  THREAD_3
} Thread_TypeDef;

/* Private variables ---------------------------------------------------------*/

osThreadId GetDataThreadId, WriteDataThreadId, ledThreadId;

osMessageQId dataQueue_id;
osMessageQDef(dataqueue, DATAQUEUE_SIZE, int);

osPoolId sensorPool_id;
osPoolDef(sensorPool, DATAQUEUE_SIZE, T_SensorsData);

osSemaphoreId readDataSem_id;
osSemaphoreDef(readDataSem);

osSemaphoreId ledBlinkSem_id;
osSemaphoreDef(ledBlinkSem);

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

static void Error_Handler( void );
void dataTimer_Callback(void const *arg);
void dataTimerStart(void);
void dataTimerStop(void);

osTimerId sensorTimId;
osTimerDef(SensorTimer, dataTimer_Callback);

uint32_t  exec;

static int32_t LSM6DSM_Sensor_IO_ITConfig( void );

volatile uint32_t * dfu_key = (volatile uint32_t*)0x10000000;

/* Private functions ---------------------------------------------------------*/
BaseType_t prvTaskReprogramCommand( int8_t *pcWriteBuffer,
                                    size_t xWriteBufferLen,
                                    const int8_t *pcCommandString )
{
    (void)xWriteBufferLen;
    vTaskList(pcWriteBuffer + 2);
    return pdFALSE;
}

static const CLI_Command_Definition_t xReprogramCommand =
{
    "reprogram",
    "reprogram: Reboots the unit in reprogramming mode",
    prvTaskReprogramCommand,
    0
};

uint8_t detectImpact(T_SensorsData *sensorData)
{
    float mag = (sensorData->acc.x * sensorData->acc.x) +
                (sensorData->acc.y * sensorData->acc.y) +
                (sensorData->acc.z * sensorData->acc.z);

    if (mag > (1300*1300))
    {
        return 1;
    }
  return 0;
}

void rebootInDFUMode()
{
    void (*SysMemBootJump)(void);
    __DSB();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
    __DSB();
    __ISB();
    SysMemBootJump = (void (*)(void)) (*((uint32_t *) 0x1fff0004)); // Point the PC to the System Memory reset vector (+4)
    __set_MSP(*(__IO uint32_t*) 0x1FFF0000);
    SysMemBootJump();
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
    // if ((dfu_key[0] == 0xdeadbeef) && (dfu_key[1] == 0x87654321) && (RCC->CSR & (1 << 28)))
    // {
    //     dfu_key[0] = 0;
    //     dfu_key[1] = 0;
    //     rebootInDFUMode();
    // }
    // dfu_key[0] = 0;
    // dfu_key[1] = 0;
    SET_BIT(RCC->CSR, 1 << 23);
    HAL_Init();

    /* Configure the System clock to 80 MHz */
    SystemClock_Config();

    i2c_expander_data i2c_cfg;
    i2c_cfg.deviceAddress = 0x40; // Includes w/r bit
    i2c_cfg.inputPorts = 0x40;
    i2c_expander_init(&i2c_cfg);
    i2c_lora_reset(1);
    i2c_lora_reset(0);
    i2c_sd_cs(1);
    i2c_sd_cs(0);
    i2c_big_led(0);
    i2c_big_led(1);

    volatile uint8_t pg = i2c_get_pg();

    /* Initialize LED */
    BSP_LED_Init(LED1);
    BSP_LED_Off(LED1);

    /* enable USB power on Pwrctrl CR2 register */
    HAL_PWREx_EnableVddUSB();
    HAL_PWREx_EnableVddIO2();

    //if(LoggingInterface == USB_Datalog) /* Configure the USB */
    {
        /*** USB CDC Configuration ***/
        /* Init Device Library */
        USBD_Init(&USBD_Device, &VCP_Desc, 0);
        /* Add Supported Class */
        USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
        /* Add Interface callbacks for AUDIO and CDC Class */
        USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
        /* Start Device Process */
        USBD_Start(&USBD_Device);
    }
    //else /* Configure the SDCard */
    {
        DATALOG_SD_Init();
    }

    osThreadDef(THREAD_1, GetData_Thread, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE*4);
    osThreadDef(THREAD_2, WriteData_Thread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*32);
    osThreadDef(THREAD_3, blinkLedThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

    GetDataThreadId = osThreadCreate(osThread(THREAD_1), NULL);
    WriteDataThreadId = osThreadCreate(osThread(THREAD_2), NULL);
    ledThreadId = osThreadCreate(osThread(THREAD_3), NULL);

    /* Register CLI info */
    FreeRTOS_CLIRegisterCommand(&xReprogramCommand);

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
    int size;
    char data_s[256];
    uint32_t impactTimer = 0;
    uint8_t SD_Log_Enabled = 0;
    uint8_t impactDetected = 0;
    volatile uint32_t * otg_dsts = (volatile uint32_t*)(0x50000808); // Can't find a #define

    for (;;)
    {
        evt = osMessageGet(dataQueue_id, osWaitForever);  // wait for message
        rptr = evt.value.p;

        // Check to see if we had large change in acceleration
        if (SD_Log_Enabled == 0)
        {
            // Start recording
            while(SD_Log_Enabled != 1)
            {
                i2c_big_led(0);
                if(DATALOG_SD_Log_Enable())
                {
                    SD_Log_Enabled=1;
                    osDelay(100);
                    //dataTimerStart();
                }
                else
                {
                    i2c_big_led(1);
                    //DATALOG_SD_Log_Disable();
                    DATALOG_SD_DeInit();
                    DATALOG_SD_Init();
                    osDelay(100);
                }
                checkLora();
            }
        }

        if (detectImpact(rptr))
        {
            // uint32_t frameSOF = (*otg_dsts >> 8) & 0x3FFF;
            // if ((impactDetected == 1) && (frameSOF > 0))
            // {
            //     strcpy(data_s, "Impact 2\n");
            //     CDC_Fill_Buffer(( uint8_t * )data_s, size);
            //     osDelay(100);
            //     dfu_key[0] = 0xdeadbeef;
            //     dfu_key[1] = 0x87654321;
            //     HAL_NVIC_SystemReset();
            // }
            // else
            // {
            //     strcpy(data_s, "Impact 1");
            //     CDC_Fill_Buffer(( uint8_t * )data_s, size);
            // }

            // Set a timer to roll the SD card recording
            impactTimer = HAL_GetTick();
            impactDetected = 1;
            osSignalSet(ledThreadId, 0x10000);

        }
        else if (SD_Log_Enabled && impactDetected && ((HAL_GetTick() - impactTimer) > 5000)) // 5 seconds
        {
            DATALOG_SD_Log_Disable();
            SD_Log_Enabled = 0;
            impactDetected = 0;
        }

        size = sprintf(data_s, "%ld, %d, %d, %d, %d, %d, %d, %d, %d, %d, %5.2f, %5.2f, %4.1f\r\n",
                    rptr->ms_counter,
                    (int)rptr->acc.x, (int)rptr->acc.y, (int)rptr->acc.z,
                    (int)rptr->gyro.x, (int)rptr->gyro.y, (int)rptr->gyro.z,
                    (int)rptr->mag.x, (int)rptr->mag.y, (int)rptr->mag.z,
                    rptr->pressure, rptr->temperature, rptr->humidity);
        osStatus status = osPoolFree(sensorPool_id, rptr);      // free memory allocated for message
        if (status < 0)
        {
            Error_Handler();
        }

        CDC_Fill_Buffer(( uint8_t * )data_s, size);
        if (SD_Log_Enabled == 1)
        {
            DATALOG_SD_writeBuf(data_s, size);
        }
    }
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

#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100

static const int8_t * const pcWelcomeMessage =
  "FreeRTOS command server.rnType Help to view a list of registered commands.rn";

#if 0
void vCommandConsoleTask( void *pvParameters )
{
Peripheral_Descriptor_t xConsole;
int8_t cRxedChar, cInputIndex = 0;
BaseType_t xMoreDataToFollow;
/* The input and output buffers are declared static to keep them off the stack. */
static int8_t pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */
    xConsole = ( Peripheral_Descriptor_t ) pvParameters;

    /* Send a welcome message to the user knows they are connected. */
    FreeRTOS_write( xConsole, pcWelcomeMessage, strlen( pcWelcomeMessage ) );

    for( ;; )
    {
        /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */
        FreeRTOS_read( xConsole, &cRxedChar, sizeof( cRxedChar ) );

        if( cRxedChar == '\n' )
        {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            FreeRTOS_write( xConsole, "\r\n", strlen( "\r\n" );

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            exaplanation of why this is. */
            do
            {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand
                              (
                                  pcInputString,   /* The command string.*/
                                  pcOutputString,  /* The output buffer. */
                                  MAX_OUTPUT_LENGTH/* The size of the output buffer. */
                              );

                /* Write the output generated by the command interpreter to the
                console. */
                FreeRTOS_write( xConsole, pcOutputString, strlen( pcOutputString ) );

            } while( xMoreDataToFollow != pdFALSE );

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            cInputIndex = 0;
            memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
        }
        else
        {
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

            if( cRxedChar == '\r' )
            {
                /* Ignore carriage returns. */
            }
            else if( cRxedChar == '\b' )
            {
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if( cInputIndex > 0 )
                {
                    cInputIndex--;
                    pcInputString[ cInputIndex ] = '';
                }
            }
            else
            {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a n is entered the complete
                string will be passed to the command interpreter. */
                if( cInputIndex < MAX_INPUT_LENGTH )
                {
                    pcInputString[ cInputIndex ] = cRxedChar;
                    cInputIndex++;
                }
            }
        }
    }
}
#endif

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

void checkLora()
{
    LORA_init();
    volatile int reg  = LORA_ReadReg(0x42);
    if (reg < 0)
    {
        // Bad things happened
        reg = 0;
    }
    else
    {
        // Good things happened
        reg++;
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

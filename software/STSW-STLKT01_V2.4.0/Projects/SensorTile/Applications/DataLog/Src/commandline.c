
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS_CLI.h"
#include "usbd_cdc_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DATAQUEUE_SIZE     ((uint32_t)100)


/* Private variables ---------------------------------------------------------*/

osMessageQId usbQueue_id;
osMessageQDef(usbQueue, DATAQUEUE_SIZE, char);

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* Reprogram -----------------------------------------------------------------*/
BaseType_t reprogramCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    const char * str1 = "Waiting for firmware download\r\n";
    const char * str2 = "Received firmware image successfully\r\n";
    static int state = 0;
    if (state == 0)
    {
        state++;
        strncpy(pcWriteBuffer, str1, xWriteBufferLen);
        return pdTRUE;
    }
    else if (state == 1)
    {
        state = 0;
        strncpy(pcWriteBuffer, str2, xWriteBufferLen);
    }
    return pdFALSE;
}

static const CLI_Command_Definition_t xReprogramCommand =
{
    "reprogram",
    "reprogram: Readies the unit for a firmware download\r\n",
    reprogramCommand,
    0
};

/* Status --------------------------------------------------------------------*/
char * state2Str(eTaskState state)
{
    switch(state)
    {
        case eRunning:
            return "X";
        case eReady:
            return "R";
        case eBlocked:
            return "B";
        case eSuspended:
            return "S";
        case eDeleted:
            return "D";
        case eInvalid:
            return "I";
        default:
            return "U";
    }
}

void formatStatus(char * str, uint32_t maxLen, TaskStatus_t *status)
{
    snprintf(str, maxLen, "%16s %s %u\r\n",
             status->pcTaskName,
             state2Str(status->eCurrentState),
             status->usStackHighWaterMark);
}

BaseType_t statusCommand( char *pcWriteBuffer,
                          size_t xWriteBufferLen,
                          const char *pcCommandString )
{
    char str[100];
    static int state = 0;
    static TaskStatus_t *pxTaskStatusArray;
    static uint32_t numTasks;
    volatile UBaseType_t uxArraySize;
    static unsigned long ulTotalRunTime, ulStatsAsPercentage;

    if (state == 0)
    {
        numTasks = uxTaskGetNumberOfTasks();
        pxTaskStatusArray = pvPortMalloc(numTasks * sizeof(TaskStatus_t));
        if (pxTaskStatusArray != NULL)
        {
            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,
                                               uxArraySize,
                                               &ulTotalRunTime);
            formatStatus(str, 100, &pxTaskStatusArray[state]);
            strncpy(pcWriteBuffer, str, xWriteBufferLen);
            state++;
            return pdTRUE;
        }
    }
    formatStatus(str, 100, &pxTaskStatusArray[state]);
    strncpy(pcWriteBuffer, str, xWriteBufferLen);
    state++;
    if (state >= numTasks)
    {
        vPortFree(pxTaskStatusArray);
        state = 0;
        return pdFALSE;
    }

    return pdTRUE;
}

static const CLI_Command_Definition_t xSystemStatus =
{
    "status",
    "status: Prints system status\r\n",
    statusCommand,
    0
};

/* Command line handler ---------------------------------------------------------*/
#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100

static const char * const pcWelcomeMessage =
  "GongBuddy command server\r\nType Help to view a list of registered commands\r\n";

void vCommandConsoleTask( const void *pvParameters )
{
    //Peripheral_Descriptor_t xConsole;
    osEvent evt;
    int8_t cRxedChar, cInputIndex = 0;
    BaseType_t xMoreDataToFollow;
    /* The input and output buffers are declared static to keep them off the stack. */
    static char pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

    /* Register CLI info */
    FreeRTOS_CLIRegisterCommand(&xReprogramCommand);
    FreeRTOS_CLIRegisterCommand(&xSystemStatus);

    usbQueue_id = osMessageCreate(osMessageQ(usbQueue), NULL);

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */
    //xConsole = ( Peripheral_Descriptor_t ) pvParameters;

    /* Send a welcome message to the user knows they are connected. */
    //FreeRTOS_write( xConsole, pcWelcomeMessage, strlen( pcWelcomeMessage ) );
    CDC_Fill_Buffer(( uint8_t * )pcWelcomeMessage, strlen( pcWelcomeMessage ));

    for( ;; )
    {
        /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */
        //FreeRTOS_read( xConsole, &cRxedChar, sizeof( cRxedChar ) );
        evt = osMessageGet(usbQueue_id, osWaitForever);  // wait for message
        cRxedChar = evt.value.v;

        if( cRxedChar == '\n' )
        {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            //FreeRTOS_write( xConsole, "\r\n", strlen( "\r\n" );
            CDC_Fill_Buffer(( uint8_t * )"\r\n", strlen("\r\n"));

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
                //FreeRTOS_write( xConsole, pcOutputString, strlen( pcOutputString ) );
                CDC_Fill_Buffer((uint8_t*)pcOutputString, strlen( pcOutputString ) );

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
                    pcInputString[ cInputIndex ] = ' ';
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
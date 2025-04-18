#pragma GCC optimize("O0")

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "low_power_manager.h"
#include "SensorTile.h"
#include "common_structs.h"
#include "cmsis_os.h"
#include "sx1276Regs-LoRa.h"
#include "utilities_conf.h"

#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             14        // dBm
#define USE_MODEM_LORA                              1

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25000     // Hz
#define FSK_DATARATE                                50000     // bps
#define FSK_BANDWIDTH                               50000     // Hz
#define FSK_AFC_BANDWIDTH                           83333     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
#error "Please define a modem in the compiler options."
#endif

typedef enum
{
  LOWPOWER,
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
} States_t;

#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here
#define LED_PERIOD_MS               200

#define LEDS_OFF   do{ \
                   LED_Off( LED_BLUE ) ;   \
                   LED_Off( LED_RED ) ;    \
                   LED_Off( LED_GREEN1 ) ; \
                   LED_Off( LED_GREEN2 ) ; \
                   } while(0) ;

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = TX;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/* Led Timers objects*/
//static  TimerEvent_t timerLed;
// Defined in main.c
extern osMessageQId loraQueue_id;
extern osPoolId loraPool_id;
// osTimerId timerTickId;
// osTimerDef(TimerTick, TimerIrqHandler);


/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

/*!
 * \brief Function executed on when led timer elapses
 */
//static void OnledEvent(void *context);
/**
 * Main application entry point.
 */
void LoRa_init()
{
  /*Disbale Stand-by mode*/
  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);

  /* Led Timers*/
  //TimerInit(&timerLed, OnledEvent);
  //TimerSetValue(&timerLed, LED_PERIOD_MS);

  //TimerStart(&timerLed);

  // Radio initialization
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.IoInit();
  Radio.Init(&RadioEvents);

  Radio.SetChannel(RF_FREQUENCY);

#if defined( USE_MODEM_LORA )

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

#elif defined( USE_MODEM_FSK )

  Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                    FSK_DATARATE, 0,
                    FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, 0, 3000);

  Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                    0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                    0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                    0, 0, false, true);

#else
#error "Please define a frequency band in the compiler options."
#endif

  Radio.Rx(RX_TIMEOUT_VALUE);
}

int poll(uint8_t flag, int timeout)
{
  TickType_t start = xTaskGetTickCount();
  TickType_t stop  = xTaskGetTickCount();

  uint8_t flags = Radio.Read(REG_LR_IRQFLAGS);
  while (((flags & flag) == 0) && ((stop - start) < timeout))
  {
    flags = Radio.Read(REG_LR_IRQFLAGS);
    stop = xTaskGetTickCount();
  }

  if ((flags & flag) == flag)
  {
    return true;
  }
  return false;
}

extern void SX1276OnDio0Irq( void* context );
int LoRa_dataexchange(uint8_t * txData,
                      uint16_t txDataLen,
                      uint8_t * rxData,
                      uint16_t rxDataBufSize)
{
  int retSize = 0;
  // Send the next PING frame
  State = TX;
  Radio.Write(REG_LR_IRQFLAGS, 0xFF);
  Radio.Send(txData, txDataLen);

  if (poll(RFLR_IRQFLAGS_TXDONE, 100) > 0)
  {
    DBGPRINTF("Successfully transmitted LoRa\n");
  }
  else
  {
    DBGPRINTF("Failed to transmitted LoRa\n");
  }

  Radio.Write(REG_LR_IRQFLAGS, 0xFF);
  Radio.Rx(0);

  if (poll(RFLR_IRQFLAGS_RXDONE, 500) > 0)
  {
    SX1276OnDio0Irq(NULL);

    DBGPRINTF("Received %d bytes\n", BufferSize);

    memcpy(rxData, Buffer, BufferSize);
    retSize = BufferSize;
  }
  else
  {
    Radio.IoInit();
    DBGPRINTF("Failed to receive LoRa\n");
  }

  //Radio.Sleep();

  return retSize;
}

#if 0
  while (1)
  {
    switch (State)
    {
      case RX:
        if (isMaster == true)
        {
          if (BufferSize > 0)
          {
            if (strncmp((const char *)Buffer, (const char *)PongMsg, 4) == 0)
            {
              TimerStop(&timerLed);
              //BSP_LED_Toggle(LED1);
              // LED_Off(LED_BLUE);
              // LED_Off(LED_GREEN) ;
              // LED_Off(LED_RED1) ;;
              // Indicates on a LED that the received frame is a PONG
              //LED_Toggle(LED_RED2) ;

              // Send the next PING frame
              Buffer[0] = 'P';
              Buffer[1] = 'I';
              Buffer[2] = 'N';
              Buffer[3] = 'G';
              // We fill the buffer with numbers for the payload
              for (i = 4; i < BufferSize; i++)
              {
                Buffer[i] = i - 4;
              }
              PRINTF("...PING\n\r");

              DelayMs(1);
              Radio.Send(Buffer, BufferSize);
            }
            else if (strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
            {
              // A master already exists then become a slave
              isMaster = false;
              //GpioWrite( &Led2, 1 ); // Set LED off
              Radio.Rx(RX_TIMEOUT_VALUE);
            }
            else // valid reception but neither a PING or a PONG message
            {
              // Set device as master ans start again
              isMaster = true;
              Radio.Rx(RX_TIMEOUT_VALUE);
            }
          }
        }
        else
        {
          if (BufferSize > 0)
          {
            if (strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
            {
              // Indicates on a LED that the received frame is a PING
              TimerStop(&timerLed);
              //BSP_LED_Toggle(LED1);

              // LED_Off(LED_RED1);
              // LED_Off(LED_RED2) ;
              // LED_Off(LED_GREEN) ;
              // LED_Toggle(LED_BLUE);

              // Send the reply to the PONG string
              Buffer[0] = 'P';
              Buffer[1] = 'O';
              Buffer[2] = 'N';
              Buffer[3] = 'G';
              // We fill the buffer with numbers for the payload
              for (i = 4; i < BufferSize; i++)
              {
                Buffer[i] = i - 4;
              }
              DelayMs(1);
              Radio.Send(Buffer, BufferSize);

              PRINTF("...PONG\n\r");
            }
            else // valid reception but not a PING as expected
            {
              // Set device as master and start again
              isMaster = true;
              Radio.Rx(RX_TIMEOUT_VALUE);
            }
          }
        }
        State = LOWPOWER;
        break;
      case TX:
        // Indicates on a LED that we have sent a PING [Master]
        // Indicates on a LED that we have sent a PONG [Slave]
        //GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );
        Radio.Rx(RX_TIMEOUT_VALUE);
        State = LOWPOWER;
        break;
      case RX_TIMEOUT:
      case RX_ERROR:
        if (isMaster == true)
        {
          // Send the next PING frame
          Buffer[0] = 'P';
          Buffer[1] = 'I';
          Buffer[2] = 'N';
          Buffer[3] = 'G';
          for (i = 4; i < BufferSize; i++)
          {
            Buffer[i] = i - 4;
          }
          DelayMs(1);
          Radio.Send(Buffer, BufferSize);
        }
        else
        {
          Radio.Rx(RX_TIMEOUT_VALUE);
        }
        State = LOWPOWER;
        break;
      case TX_TIMEOUT:
        Radio.Rx(RX_TIMEOUT_VALUE);
        State = LOWPOWER;
        break;
      case LOWPOWER:
      default:
        // Set low power
        break;
    }

    DISABLE_IRQ();
    /* if an interupt has occured after __disable_irq, it is kept pending
     * and cortex will not enter low power anyway  */
    if (State == LOWPOWER)
    {
#ifndef LOW_POWER_DISABLE
      LPM_EnterLowPower();
#endif
    }
    ENABLE_IRQ();
  }
}
#endif

void OnTxDone(void)
{
  Radio.Sleep();
  State = TX;
  PRINTF("OnTxDone\n\r");
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  Radio.Sleep();
  BufferSize = size;
  memcpy(Buffer, payload, BufferSize);
  RssiValue = rssi;
  SnrValue = snr;
  State = RX;

  PRINTF("OnRxDone\n\r");
  PRINTF("RssiValue=%d dBm, SnrValue=%d\n\r", rssi, snr);
}

void OnTxTimeout(void)
{
  Radio.Sleep();
  State = TX_TIMEOUT;

  PRINTF("OnTxTimeout\n\r");
}

void OnRxTimeout(void)
{
  Radio.Sleep();
  State = RX_TIMEOUT;
  PRINTF("OnRxTimeout\n\r");
}

void OnRxError(void)
{
  Radio.Sleep();
  State = RX_ERROR;
  PRINTF("OnRxError\n\r");
}

// static void OnledEvent(void *context)
// {
//   //BSP_LED_Toggle(LED1);

//   // LED_Toggle(LED_BLUE) ;
//   // LED_Toggle(LED_RED1) ;
//   // LED_Toggle(LED_RED2) ;
//   // LED_Toggle(LED_GREEN) ;

//   TimerStart(&timerLed);
// }

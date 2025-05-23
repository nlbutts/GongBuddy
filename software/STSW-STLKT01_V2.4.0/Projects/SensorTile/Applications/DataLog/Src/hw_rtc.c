/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: MCU RTC timer

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    hw_rtc.c
  * @author  MCD Application Team
  * @brief   driver for RTC
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <time.h>
#include "hw.h"
#include "low_power_manager.h"
#include "systime.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint32_t  Rtc_Time; /* Reference time */

  RTC_TimeTypeDef RTC_Calndr_Time; /* Reference time in calendar format */

  RTC_DateTypeDef RTC_Calndr_Date; /* Reference date in calendar format */

} RtcTimerContext_t;

/* Private define ------------------------------------------------------------*/

/* MCU Wake Up Time */
#define MIN_ALARM_DELAY               3 /* in ticks */

/* subsecond number of bits */
#define N_PREDIV_S                 10

/* Synchonuous prediv  */
#define PREDIV_S                  ((1<<N_PREDIV_S)-1)

/* Asynchonuous prediv   */
#define PREDIV_A                  (1<<(15-N_PREDIV_S))-1

/* Sub-second mask definition  */
#define HW_RTC_ALARMSUBSECONDMASK (N_PREDIV_S<<RTC_ALRMASSR_MASKSS_Pos)

/* RTC Time base in us */
#define USEC_NUMBER               1000000
#define MSEC_NUMBER               (USEC_NUMBER/1000)
#define RTC_ALARM_TIME_BASE       (USEC_NUMBER>>N_PREDIV_S)

#define COMMON_FACTOR        3
#define CONV_NUMER                (MSEC_NUMBER>>COMMON_FACTOR)
#define CONV_DENOM                (1<<(N_PREDIV_S-COMMON_FACTOR))

#define DAYS_IN_LEAP_YEAR                        ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                             ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                          ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                         ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                       ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                         ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                            ( ( uint32_t )   24U )

#define  DAYS_IN_MONTH_CORRECTION_NORM     ((uint32_t) 0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP     ((uint32_t) 0x445550 )

#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*!
 * \brief Indicates if the RTC is already Initalized or not
 */
static bool HW_RTC_Initalized = false;

/*!
 * \brief compensates MCU wakeup time
 */

static bool McuWakeUpTimeInitialized = false;

/*!
 * \brief compensates MCU wakeup time
 */

static int16_t McuWakeUpTimeCal = 0;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static RTC_HandleTypeDef RtcHandle = {0};

static RTC_AlarmTypeDef RTC_AlarmStructure;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the HW_RTC_SetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/* Private function prototypes -----------------------------------------------*/

static void HW_RTC_SetConfig(void);

static void HW_RTC_SetAlarmConfig(void);

static void HW_RTC_StartWakeUpAlarm(uint32_t timeoutValue);

static uint64_t HW_RTC_GetCalendarValue(RTC_DateTypeDef *RTC_DateStruct, RTC_TimeTypeDef *RTC_TimeStruct);


/* Exported functions ---------------------------------------------------------*/

/*!
 * @brief Initializes the RTC timer
 * @note The timer is based on the RTC
 * @param none
 * @retval none
 */
void HW_RTC_Init(void)
{
  if (HW_RTC_Initalized == false)
  {
    HW_RTC_SetConfig();
    HW_RTC_SetAlarmConfig();
    HW_RTC_SetTimerContext();
    HW_RTC_Initalized = true;
  }
}

/*!
 * @brief Configures the RTC timer
 * @note The timer is based on the RTC
 * @param none
 * @retval none
 */
static void HW_RTC_SetConfig(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  RtcHandle.Instance = RTC;

  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv = PREDIV_A; /* RTC_ASYNCH_PREDIV; */
  RtcHandle.Init.SynchPrediv = PREDIV_S; /* RTC_SYNCH_PREDIV; */
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  HAL_RTC_Init(&RtcHandle);

  /*Monday 1st January 2016*/
  RTC_DateStruct.Year = 0;
  RTC_DateStruct.Month = RTC_MONTH_JANUARY;
  RTC_DateStruct.Date = 1;
  RTC_DateStruct.WeekDay = RTC_WEEKDAY_MONDAY;
  HAL_RTC_SetDate(&RtcHandle, &RTC_DateStruct, RTC_FORMAT_BIN);

  /*at 0:0:0*/
  RTC_TimeStruct.Hours = 0;
  RTC_TimeStruct.Minutes = 0;

  RTC_TimeStruct.Seconds = 0;
  RTC_TimeStruct.TimeFormat = 0;
  RTC_TimeStruct.SubSeconds = 0;
  RTC_TimeStruct.StoreOperation = RTC_DAYLIGHTSAVING_NONE;
  RTC_TimeStruct.DayLightSaving = RTC_STOREOPERATION_RESET;

  HAL_RTC_SetTime(&RtcHandle, &RTC_TimeStruct, RTC_FORMAT_BIN);

  /*Enable Direct Read of the calendar registers (not through Shadow) */
  HAL_RTCEx_EnableBypassShadow(&RtcHandle);
}


/*!
 * @brief calculates the wake up time between wake up and mcu start
 * @note resulotion in RTC_ALARM_TIME_BASE in timer ticks
 * @param none
 * @retval none
 */
void HW_RTC_setMcuWakeUpTime(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  TimerTime_t now, hit;
  int16_t McuWakeUpTime;

  if ((McuWakeUpTimeInitialized == false) &&
      (HAL_NVIC_GetPendingIRQ(RTC_Alarm_IRQn) == 1))
  {
    /* warning: works ok if now is below 30 days
       it is ok since it's done once at first alarm wake-up*/
    McuWakeUpTimeInitialized = true;
    now = (uint32_t) HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

    HAL_RTC_GetAlarm(&RtcHandle, &RTC_AlarmStructure, RTC_ALARM_A, RTC_FORMAT_BIN);
    hit = RTC_AlarmStructure.AlarmTime.Seconds +
          60 * (RTC_AlarmStructure.AlarmTime.Minutes +
                60 * (RTC_AlarmStructure.AlarmTime.Hours +
                      24 * (RTC_AlarmStructure.AlarmDateWeekDay)));
    hit = (hit << N_PREDIV_S) + (PREDIV_S - RTC_AlarmStructure.AlarmTime.SubSeconds);

    McuWakeUpTime = (int16_t)((now - hit));
    McuWakeUpTimeCal += McuWakeUpTime;
  }
}

int16_t HW_RTC_getMcuWakeUpTime(void)
{
  return McuWakeUpTimeCal;
}

/*!
 * @brief returns the wake up time in ticks
 * @param none
 * @retval wake up time in ticks
 */
uint32_t HW_RTC_GetMinimumTimeout(void)
{
  return (MIN_ALARM_DELAY);
}

/*!
 * @brief converts time in ms to time in ticks
 * @param [IN] time in milliseconds
 * @retval returns time in timer ticks
 */
uint32_t HW_RTC_ms2Tick(TimerTime_t timeMilliSec)
{
  /*return( ( timeMicroSec / RTC_ALARM_TIME_BASE ) ); */
  return (uint32_t)((((uint64_t)timeMilliSec) * CONV_DENOM) / CONV_NUMER);
}

/*!
 * @brief converts time in ticks to time in ms
 * @param [IN] time in timer ticks
 * @retval returns time in milliseconds
 */
TimerTime_t HW_RTC_Tick2ms(uint32_t tick)
{
  /*return( ( timeMicroSec * RTC_ALARM_TIME_BASE ) ); */
  uint32_t seconds = tick >> N_PREDIV_S;
  tick = tick & PREDIV_S;
  return ((seconds * 1000) + ((tick * 1000) >> N_PREDIV_S));
}

/*!
 * @brief Set the alarm
 * @note The alarm is set at now (read in this funtion) + timeout
 * @param timeout Duration of the Timer ticks
 */
void HW_RTC_SetAlarm(uint32_t timeout)
{
  /* we don't go in Low Power mode for timeout below MIN_ALARM_DELAY */
  if ((MIN_ALARM_DELAY + McuWakeUpTimeCal) < ((timeout - HW_RTC_GetTimerElapsedTime())))
  {
    LPM_SetStopMode(LPM_RTC_Id, LPM_Enable);
  }
  else
  {
    LPM_SetStopMode(LPM_RTC_Id, LPM_Disable);
  }

  /*In case stop mode is required */
  if (LPM_GetMode() == LPM_StopMode)
  {
    timeout = timeout -  McuWakeUpTimeCal;
  }

  HW_RTC_StartWakeUpAlarm(timeout);
}

/*!
 * @brief Get the RTC timer elapsed time since the last Alarm was set
 * @param none
 * @retval RTC Elapsed time in ticks
 */
uint32_t HW_RTC_GetTimerElapsedTime(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  uint32_t CalendarValue = (uint32_t) HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

  return ((uint32_t)(CalendarValue - RtcTimerContext.Rtc_Time));
}

/*!
 * @brief Get the RTC timer value
 * @param none
 * @retval RTC Timer value in ticks
 */
uint32_t HW_RTC_GetTimerValue(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  uint32_t CalendarValue = (uint32_t) HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

  return (CalendarValue);
}

/*!
 * @brief Stop the Alarm
 * @param none
 * @retval none
 */
void HW_RTC_StopAlarm(void)
{
  /* Disable the Alarm A interrupt */
  HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
  /* Clear RTC Alarm Flag */
  __HAL_RTC_ALARM_CLEAR_FLAG(&RtcHandle, RTC_FLAG_ALRAF);
  /* Clear the EXTI's line Flag for RTC Alarm */
  __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
}

/*!
 * @brief RTC IRQ Handler on the RTC Alarm
 * @param none
 * @retval none
 */
void HW_RTC_IrqHandler(void)
{
  RTC_HandleTypeDef *hrtc = &RtcHandle;
  /* enable low power at irq*/
  LPM_SetStopMode(LPM_RTC_Id, LPM_Enable);

  /* Clear the EXTI's line Flag for RTC Alarm */
  __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

  /* Get the AlarmA interrupt source enable status */
  if (__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_ALRA) != RESET)
  {
    /* Get the pending status of the AlarmA Interrupt */
    if (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF) != RESET)
    {
      /* Clear the AlarmA interrupt pending bit */
      __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
      /* AlarmA callback */
      HAL_RTC_AlarmAEventCallback(hrtc);
    }
  }
}


/*!
 * @brief a delay of delay ms by polling RTC
 * @param delay in ms
 * @retval none
 */
void HW_RTC_DelayMs(uint32_t delay)
{
  TimerTime_t delayValue = 0;
  TimerTime_t timeout = 0;

  delayValue = HW_RTC_ms2Tick(delay);

  /* Wait delay ms */
  timeout = HW_RTC_GetTimerValue();
  while (((HW_RTC_GetTimerValue() - timeout)) < delayValue)
  {
    __NOP();
  }
}

/*!
 * @brief set Time Reference set also the RTC_DateStruct and RTC_TimeStruct
 * @param none
 * @retval Timer Value
 */
uint32_t HW_RTC_SetTimerContext(void)
{
  RtcTimerContext.Rtc_Time = (uint32_t) HW_RTC_GetCalendarValue(&RtcTimerContext.RTC_Calndr_Date, &RtcTimerContext.RTC_Calndr_Time);
  return (uint32_t) RtcTimerContext.Rtc_Time;
}

/*!
 * @brief Get the RTC timer Reference
 * @param none
 * @retval Timer Value in  Ticks
 */
uint32_t HW_RTC_GetTimerContext(void)
{
  return RtcTimerContext.Rtc_Time;
}
/* Private functions ---------------------------------------------------------*/

/*!
 * @brief configure alarm at init
 * @param none
 * @retval none
 */
static void HW_RTC_SetAlarmConfig(void)
{
  HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
}

/*!
 * @brief start wake up alarm
 * @note  alarm in RtcTimerContext.Rtc_Time + timeoutValue
 * @param timeoutValue in ticks
 * @retval none
 */
static void HW_RTC_StartWakeUpAlarm(uint32_t timeoutValue)
{
  uint16_t rtcAlarmSubSeconds = 0;
  uint16_t rtcAlarmSeconds = 0;
  uint16_t rtcAlarmMinutes = 0;
  uint16_t rtcAlarmHours = 0;
  uint16_t rtcAlarmDays = 0;
  RTC_TimeTypeDef RTC_TimeStruct = RtcTimerContext.RTC_Calndr_Time;
  RTC_DateTypeDef RTC_DateStruct = RtcTimerContext.RTC_Calndr_Date;

  HW_RTC_StopAlarm();

  /*reverse counter */
  rtcAlarmSubSeconds =  PREDIV_S - RTC_TimeStruct.SubSeconds;
  rtcAlarmSubSeconds += (timeoutValue & PREDIV_S);
  /* convert timeout  to seconds */
  timeoutValue >>= N_PREDIV_S;  /* convert timeout  in seconds */

  /*convert microsecs to RTC format and add to 'Now' */
  rtcAlarmDays =  RTC_DateStruct.Date;
  while (timeoutValue >= SECONDS_IN_1DAY)
  {
    timeoutValue -= SECONDS_IN_1DAY;
    rtcAlarmDays++;
  }

  /* calc hours */
  rtcAlarmHours = RTC_TimeStruct.Hours;
  while (timeoutValue >= SECONDS_IN_1HOUR)
  {
    timeoutValue -= SECONDS_IN_1HOUR;
    rtcAlarmHours++;
  }

  /* calc minutes */
  rtcAlarmMinutes = RTC_TimeStruct.Minutes;
  while (timeoutValue >= SECONDS_IN_1MINUTE)
  {
    timeoutValue -= SECONDS_IN_1MINUTE;
    rtcAlarmMinutes++;
  }

  /* calc seconds */
  rtcAlarmSeconds =  RTC_TimeStruct.Seconds + timeoutValue;

  /***** correct for modulo********/
  while (rtcAlarmSubSeconds >= (PREDIV_S + 1))
  {
    rtcAlarmSubSeconds -= (PREDIV_S + 1);
    rtcAlarmSeconds++;
  }

  while (rtcAlarmSeconds >= SECONDS_IN_1MINUTE)
  {
    rtcAlarmSeconds -= SECONDS_IN_1MINUTE;
    rtcAlarmMinutes++;
  }

  while (rtcAlarmMinutes >= MINUTES_IN_1HOUR)
  {
    rtcAlarmMinutes -= MINUTES_IN_1HOUR;
    rtcAlarmHours++;
  }

  while (rtcAlarmHours >= HOURS_IN_1DAY)
  {
    rtcAlarmHours -= HOURS_IN_1DAY;
    rtcAlarmDays++;
  }

  if (RTC_DateStruct.Year % 4 == 0)
  {
    if (rtcAlarmDays > DaysInMonthLeapYear[ RTC_DateStruct.Month - 1 ])
    {
      rtcAlarmDays = rtcAlarmDays % DaysInMonthLeapYear[ RTC_DateStruct.Month - 1 ];
    }
  }
  else
  {
    if (rtcAlarmDays > DaysInMonth[ RTC_DateStruct.Month - 1 ])
    {
      rtcAlarmDays = rtcAlarmDays % DaysInMonth[ RTC_DateStruct.Month - 1 ];
    }
  }

  /* Set RTC_AlarmStructure with calculated values*/
  RTC_AlarmStructure.AlarmTime.SubSeconds = PREDIV_S - rtcAlarmSubSeconds;
  RTC_AlarmStructure.AlarmSubSecondMask  = HW_RTC_ALARMSUBSECONDMASK;
  RTC_AlarmStructure.AlarmTime.Seconds = rtcAlarmSeconds;
  RTC_AlarmStructure.AlarmTime.Minutes = rtcAlarmMinutes;
  RTC_AlarmStructure.AlarmTime.Hours   = rtcAlarmHours;
  RTC_AlarmStructure.AlarmDateWeekDay    = (uint8_t)rtcAlarmDays;
  RTC_AlarmStructure.AlarmTime.TimeFormat   = RTC_TimeStruct.TimeFormat;
  RTC_AlarmStructure.AlarmDateWeekDaySel   = RTC_ALARMDATEWEEKDAYSEL_DATE;
  RTC_AlarmStructure.AlarmMask       = RTC_ALARMMASK_NONE;
  RTC_AlarmStructure.Alarm = RTC_ALARM_A;
  RTC_AlarmStructure.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  RTC_AlarmStructure.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;

  /* Set RTC_Alarm */
  HAL_RTC_SetAlarm_IT(&RtcHandle, &RTC_AlarmStructure, RTC_FORMAT_BIN);
}


/*!
 * @brief get current time from calendar in ticks
 * @param pointer to RTC_DateStruct
 * @param pointer to RTC_TimeStruct
 * @retval time in ticks
 */
static uint64_t HW_RTC_GetCalendarValue(RTC_DateTypeDef *RTC_DateStruct, RTC_TimeTypeDef *RTC_TimeStruct)
{
  uint64_t calendarValue = 0;
  uint32_t first_read;
  uint32_t correction;
  uint32_t seconds;

  /* Get Time and Date*/
  HAL_RTC_GetTime(&RtcHandle, RTC_TimeStruct, RTC_FORMAT_BIN);

  /* make sure it is correct due to asynchronus nature of RTC*/
  do
  {
    first_read = LL_RTC_TIME_GetSubSecond(RTC);
    HAL_RTC_GetDate(&RtcHandle, RTC_DateStruct, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&RtcHandle, RTC_TimeStruct, RTC_FORMAT_BIN);

  }
  while (first_read != LL_RTC_TIME_GetSubSecond(RTC));

  /* calculte amount of elapsed days since 01/01/2000 */
  seconds = DIVC((DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR) * RTC_DateStruct->Year, 4);

  correction = ((RTC_DateStruct->Year % 4) == 0) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM ;

  seconds += (DIVC((RTC_DateStruct->Month - 1) * (30 + 31), 2) - (((correction >> ((RTC_DateStruct->Month - 1) * 2)) & 0x3)));

  seconds += (RTC_DateStruct->Date - 1);

  /* convert from days to seconds */
  seconds *= SECONDS_IN_1DAY;

  seconds += ((uint32_t)RTC_TimeStruct->Seconds +
              ((uint32_t)RTC_TimeStruct->Minutes * SECONDS_IN_1MINUTE) +
              ((uint32_t)RTC_TimeStruct->Hours * SECONDS_IN_1HOUR)) ;



  calendarValue = (((uint64_t) seconds) << N_PREDIV_S) + (PREDIV_S - RTC_TimeStruct->SubSeconds);

  return (calendarValue);
}

/*!
 * \brief Get system time
 * \param [IN]   pointer to ms
 *
 * \return uint32_t seconds
 */
uint32_t HW_RTC_GetCalendarTime(uint16_t *mSeconds)
{
  RTC_TimeTypeDef RTC_TimeStruct ;
  RTC_DateTypeDef RTC_DateStruct;
  uint32_t ticks;

  uint64_t calendarValue = HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

  uint32_t seconds = (uint32_t)(calendarValue >> N_PREDIV_S);

  ticks = (uint32_t) calendarValue & PREDIV_S;

  *mSeconds = HW_RTC_Tick2ms(ticks);

  return seconds;
}

void HW_RTC_BKUPWrite(uint32_t Data0, uint32_t Data1)
{
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, Data0);
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, Data1);
}

void HW_RTC_BKUPRead(uint32_t *Data0, uint32_t *Data1)
{
  *Data0 = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0);
  *Data1 = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1);
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature)
{
  float k = RTC_TEMP_COEFFICIENT;
  float kDev = RTC_TEMP_DEV_COEFFICIENT;
  float t = RTC_TEMP_TURNOVER;
  float tDev = RTC_TEMP_DEV_TURNOVER;
  float interim = 0.0;
  float ppm = 0.0;

  if (k < 0.0f)
  {
    ppm = (k - kDev);
  }
  else
  {
    ppm = (k + kDev);
  }
  interim = (temperature - (t - tDev));
  ppm *=  interim * interim;

  // Calculate the drift in time
  interim = ((float) period * ppm) / 1000000;
  // Calculate the resulting time period
  interim += period;
  interim = floor(interim);

  if (interim < 0.0f)
  {
    interim = (float)period;
  }

  // Calculate the resulting period
  return (TimerTime_t) interim;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


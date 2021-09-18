/**
 * @file Timer.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief General timer system with logging to Segger RTT
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef TIMER_H_
#define TIMER_H_

#include "ITimer.h"

class Timer : public ITimer
{
    public:
        /**
         * Timer constructor, the timer overlays a free running 1 us timer and provides
         * functionality to time events.
         *
         * @param str the name of this timer
         */
        Timer(const char * str);
        Timer();
        virtual ~Timer();

        /**
         * This function does some general initialization of the timer subsystem
         * It MUST be called before any timer modules are used
         */
        static void initialize();
        /**
         * This function blocks the CPU for the duration.
         *
         * @param timeInMs the time to block the CPU in milliseconds
         */
        void delayMs(uint32_t timeInMs);
        void delayUs(uint32_t timeInUs);
        /**
         * This function reads the hardware timer into a variable, this is useful for starting
         * timer measurements
         */
        void start();
        /**
         * This function reads the hardware timer into a variable, this is useful for stoping
         * timer measurements
         */
        void stop();
        /**
         * This reads the current timer and calculates a time duration in milliseconds
         *
         * @return uint32_t time in milliseconds
         */
        uint32_t getTimeInMs();
        /**
         * This reads the current timer and calculates a time duration in microseconds
         *
         * @return uint32_t time in microseconds
         */
        uint32_t getTimeInUs();
        /**
         * This reads the current timer and calculates a time duration in microseconds
         *
         * @return const char *  string describing the time
         */
        const char * logTimeInMs();
        /**
         * This reads the current timer and calculates a time duration in microseconds
         *
         * @return const char *  string describing the time
         */
        const char * logTimeInUs();
        /**
         * This function sets a expiration timer
         *
         * @param timeoutInMs time in milliseconds
         */
        virtual void setTimerMs(uint32_t timeoutInMs);
        virtual void setTimerUs(uint32_t timeoutInUs);
        /**
         * This function advances a timer keeping a steady period
         *
         * @param timeoutInMs time in milliseconds
         */
        void advanceTimerMs(uint32_t timeoutInMs);
        void advanceTimerUs(uint32_t timeoutInUs);
        /**
         * This function returns true if this timer has expired
         *
         * @return bool true if the timer has expired, false otherwise
         */
        virtual bool isTimerExpired() const;

        /**
         * Registers a timer with the global linked list
         */
        void registerTimer();
        /**
         * Remove this timer from the linked list
         */
        void removeTimer();

        /**
         * This function scans through the timer linked list and marks timers as expired
         */
        static void periodic();

        /**
         * Sets/gets the next timer pointer
         *
         * @param nextTimer the next timer in the linked list
         * @return Timer* pointer to the next timer
         */
        void setNextTimer(Timer * nextTimer)    {_next = nextTimer;}
        Timer * getNextTimer()                  {return _next;}

        /**
         * Marks a timer as expired
         */
        void markTimerExpired()     {_expired = true;}

    private:
        /**
         * This function generates the formatted string that indicates how much time has passed
         *
         * @param timespan the timespan in whatever units the user wants them in
         */
        void logTime(uint32_t timeSpan);

        static uint8_t               _channel;           // indicates which channel this class is using
        static uint32_t              _frequency;         // The frequency this timer is running at
        static Timer               * _root;              // Root pointer to the linked list

        const char                 * _timerDesc;         // A text description of this timer, used for timing intervals
        uint32_t                     _expireStart;       // start time of the expiration timer
        uint32_t                     _expireDuration;    // duration of the expiration timer
        uint32_t                     _start;             // the start time for a periodic timer
        uint32_t                     _stop;              // the stop time for a periodic timer
        bool                         _expired;           // Set to true if the periodic function detects this timer as expired
        Timer                      * _next;              // Pointer to the next timer in the linked list

        static const uint32_t _tempStrSize = 32;   // Maybe this should be part of the template
        char _tempStr[_tempStrSize];            // Used for printing timing intervals
};


#endif /* TIMER_H_ */

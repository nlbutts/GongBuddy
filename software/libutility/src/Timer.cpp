/**
 * @file Timer.cpp
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief General timer system with logging to Segger RTT
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <cstdio>
#include <cstring>
#include "Timer.h"

Timer::Timer(const char * str)
: _timerDesc(str)
, _expireStart(0)
, _expireDuration(0)
, _start(0)
, _stop(0)
, _expired(true)
, _next(NULL)
{
    _tempStr[0] = '\0';
    start();
    registerTimer();
}

Timer::Timer()
: _timerDesc("")
, _expireStart(0)
, _expireDuration(0)
, _start(0)
, _stop(0)
, _expired(true)
, _next(NULL)
{
    _tempStr[0] = '\0';
    start();
    registerTimer();
}

Timer::~Timer()
{
    _timerDesc = NULL;
    removeTimer();
    _next = NULL;
}

void Timer::initialize()
{
}

void Timer::delayMs(uint32_t timeInMs)
{
    delayUs(timeInMs*1000);
}

void Timer::delayUs(uint32_t timeInUs)
{
    uint32_t startDelay = *_timerReg;
    uint32_t now = startDelay;
    while ((now - startDelay) < timeInUs)
    {
        now = *_timerReg;
    }
}

void Timer::start()
{
    _start = *_timerReg;
}

void Timer::stop()
{
    _stop = *_timerReg;
}

uint32_t Timer::getTimeInMs()
{
    return getTimeInUs() / 1000;
}

uint32_t Timer::getTimeInUs()
{
    stop();
    return _stop - _start;
}


const char * Timer::logTimeInMs()
{
    logTime(getTimeInMs());
    strncat(_tempStr, " ms\n", _tempStrSize - strlen(_tempStr));
    return _tempStr;
}

const char * Timer::logTimeInUs()
{
    logTime(getTimeInUs());
    strncat(_tempStr, " us\n", _tempStrSize - strlen(_tempStr));
    return _tempStr;
}

void Timer::logTime(uint32_t timeSpan)
{
    strncpy(_tempStr, _timerDesc, _tempStrSize);
    strncat(_tempStr, " time=", _tempStrSize - strlen(_tempStr));
    char value[10];
    snprintf(value, 10, "%lu", timeSpan);   /*lint !e534 */
    strncat(_tempStr, value, _tempStrSize - strlen(_tempStr));
}

void Timer::setTimerMs(uint32_t timeoutInMs)
{
    setTimerUs(timeoutInMs * 1000);
}

void Timer::setTimerUs(uint32_t timeoutInUs)
{
    _expireStart    = *_timerReg;
    _expireDuration = timeoutInUs;
    _expired        = false;
    _start          = _expireStart;
}

void Timer::advanceTimerMs(uint32_t timeoutInMs)
{
    advanceTimerUs(timeoutInMs * 1000);
}

void Timer::advanceTimerUs(uint32_t timeoutInUs)
{
    _expireStart    += timeoutInUs;
    _start          += timeoutInUs;
    _expireDuration  = timeoutInUs;
    _expired         = false;
}

bool Timer::isTimerExpired() const
{
    uint32_t now = *_timerReg;
    if (_expired || ((now - _expireStart) >= _expireDuration))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Timer::registerTimer()
{
    Timer * currentTimer = _root;
    _root = this;
    _next = currentTimer;
}

void Timer::removeTimer()
{
    if (_root == this)
    {
        // quick test for stack timers
        _root = _next;
    }
    else
    {
        Timer * previousTimer = _root;
        Timer * currentTimer  = _root->getNextTimer();
        while (currentTimer != NULL)
        {
            if (currentTimer == this)
            {
                previousTimer->setNextTimer(_next);
                break;
            }
        }
    }
}

void Timer::periodic()
{
    Timer * currentTimer = _root;

    while (currentTimer != NULL)
    {
        if (currentTimer->isTimerExpired())
        {
            currentTimer->markTimerExpired();
        }
        currentTimer = currentTimer->getNextTimer();
    }
}

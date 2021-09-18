/**
 * @file ITimer.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef ITIMER_H_
#define ITIMER_H_

#include <stdint.h>

/**
 * Specifies the contract of a TimerDriver
 *
 */
class ITimer
{
public:
    /**
     * Destroys an instance of <code>ITimer</code>.
     */
    virtual ~ITimer() {};

    /**
     * This function sets a expiration timer
     *
     * @param timeoutInMs time in milliseconds
     */
    virtual void setTimerMs(uint32_t timeoutInMs) = 0;
    virtual void setTimerUs(uint32_t timeoutInUs) = 0;

    /**
     * @brief Delay for a certain amount of time
     *
     * @param timeInMs time in milliseconds
     */
    virtual void delayMs(uint32_t timeInMs) = 0;
    virtual void delayUs(uint32_t timeInUs) = 0;
    /**
     * This function returns true if this timer has expired
     *
     * @return bool true if the timer has expired, false otherwise
     */
    virtual bool isTimerExpired() const = 0;

protected:
    /**
     * Protected to comply with pure virtual interface.
     */
    ITimer() {};
};

#endif /* ITIMER_H_ */

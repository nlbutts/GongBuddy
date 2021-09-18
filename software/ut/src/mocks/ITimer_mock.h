/**
 * @file IFlash_mock.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief Mock for the FLASH interface
 * @version 0.1
 * @date 2021-09-12
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <gmock/gmock.h>
#include "ITimer.h"

class ITimer_mock : public ITimer {
 public:
  MOCK_METHOD(void, setTimerMs, (uint32_t timeoutInMs), (override));
  MOCK_METHOD(void, setTimerUs, (uint32_t timeoutInUs), (override));
  MOCK_METHOD(void, delayMs, (uint32_t timeoutInMs), (override));
  MOCK_METHOD(void, delayUs, (uint32_t timeoutInUs), (override));
  MOCK_METHOD(bool, isTimerExpired, (), (const, override));
};
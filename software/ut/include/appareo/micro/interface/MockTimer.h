/**
 * Copyright (c) 2017 Appareo Systems, LLC.
 * 1810 NDSU Research Park Circle North
 * Fargo ND, 58102
 * All rights reserved.
 *
 * This is the confidential and proprietary information of Appareo Systems, LLC.
 * You shall not disclose such confidential information and shall use it only in
 * accordance with the terms of the license agreement you entered into with Appareo.
 *
 * File: MockTimer.h
 * Creator: pcrowley
 * Date: May 9, 2018
 *
 * Copyright Version 1.0
 */

#include <stdint.h>
#include <gmock/gmock.h>
#include <appareo/micro/interface/ITimer.h>

namespace Appareo 	{
namespace Micro 	{
namespace Interface {
namespace Mock 		{

class MockTimer : public ITimer
{
public:
    MOCK_METHOD1(setTimerMs, void(uint32_t timeoutInMs));
    MOCK_METHOD1(setTimerUs, void(uint32_t timeoutInUs));
    MOCK_CONST_METHOD0(isTimerExpired, bool());
};

} // namespace Mock
} // namespace Interface
} // namespace Micro
} // namespace Appareo
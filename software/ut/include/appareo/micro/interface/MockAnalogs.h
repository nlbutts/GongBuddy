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
 * File: MockAnalogs.h
 * Creator: pcrowley
 * Date: May 9, 2018
 *
 * Copyright Version 1.0
 */

#include <stdint.h>
#include <gmock/gmock.h>
#include <appareo/micro/interface/IAnalogs.h>

namespace Appareo 	{
namespace Micro 	{
namespace Interface {
namespace Mock 		{

class MockAnalogs : public IAnalogs
{
public:
    MOCK_CONST_METHOD1(get, uint16_t(uint16_t rank));
    MOCK_METHOD0(start, void());
    MOCK_METHOD2(start, void(uint16_t totalConversions, uint16_t * buffer));
    MOCK_METHOD0(stop, uint16_t());
    MOCK_METHOD0(overrun, bool());
    MOCK_CONST_METHOD0(getMaxADCValue, uint16_t());
    MOCK_CONST_METHOD0(getADCVRef, float());
};

} // namespace Mock
} // namespace Interface
} // namespace Micro
} // namespace Appareo
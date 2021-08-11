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
#include <functional>
#include <appareo/micro/interface/ICapture.h>

namespace Appareo 	{
namespace Micro 	{
namespace Interface {
namespace Mock 		{

class MockCapture : public ICapture
{
public:
    void registerInterrutHandler(std::function<void(CaptureData)> func) {};
};

} // namespace Mock
} // namespace Interface
} // namespace Micro
} // namespace Appareo
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
 * File: MockGainController.h
 * Creator: pcrowley
 * Date: May 9, 2018
 *
 * Copyright Version 1.0
 */

#include <stdint.h>
#include <gmock/gmock.h>
#include <appareo/southheart/sensors/IGainController.h>

namespace Appareo 	  {
namespace SouthHeart {
namespace Sensors     {
namespace Mock 		  {

class MockGainController : public IGainController
{
public:
    MOCK_METHOD1(adjustGain, void(float percentError));
};

} // namespace Mock
} // namespace Sensors
} // namespace SouthHeart
} // namespace Appareo
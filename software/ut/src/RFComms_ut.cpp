#include <stdint.h>
#include <gtest/gtest.h>

#include "Irfcomms.h"
#include "mocks/rfcomms_mock.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;

TEST(RFComms, senddata)
{
    std::vector<uint8_t> data;
    for (auto i = 0; i < 10; i++)
    {
        data.push_back(i);
    }

    MockRFComms comms;
    EXPECT_CALL(comms, sendData(_))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(data.size()));
    int rv = comms.sendData(data);
    ASSERT_EQ(rv, data.size());
}
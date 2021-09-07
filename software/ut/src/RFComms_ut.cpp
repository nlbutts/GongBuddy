#include <stdint.h>
#include <gtest/gtest.h>

#include "Irfcomms.h"
#include "mocks/rfcomms_mock.h"

TEST(RFComms, senddata)
{
    RFComms comms;
    std::vector<uint8_t> data;
    for (auto i = 0; i < 10; i++)
    {
        data.push_back(i);
    }
    int rv = comms.sendData(data);
    ASSERT_EQ(rv, 10);
}
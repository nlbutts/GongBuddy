#include <stdint.h>
//#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "gb_messages.pb.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::Ge;
using ::testing::NotNull;
using ::testing::SetArrayArgument;
using ::testing::DoAll;

TEST(nanopb, nanopb_test)
{
    uint8_t buf[250];
    LoraMsg2 msg1 = LoraMsg2_init_default;
    LoraMsg2 msg2 = LoraMsg2_init_default;

    msg1.status = Status_IMPACT;
    // msg1.fw_setup.fw_crc = 1234;
    // msg1.fw_setup.start_address = 5678;
    // msg1.fw_setup.total_packets = 50;
    // msg1.has_fw_setup = true;

    pb_ostream_t stream1 = pb_ostream_from_buffer(buf, sizeof(buf));
    bool result;
    result = pb_encode(&stream1, LoraMsg2_fields, &msg1);
    EXPECT_TRUE(result);

    pb_istream_t stream2 = pb_istream_from_buffer(buf, stream1.bytes_written);
    result = pb_decode(&stream2, LoraMsg2_fields, &msg2);
    EXPECT_TRUE(result);

    int res = memcmp(&msg1, &msg2, sizeof(LoraMsg2));
    EXPECT_EQ(res, 0);
}
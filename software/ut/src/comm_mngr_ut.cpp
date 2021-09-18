#include <stdint.h>
#include <gtest/gtest.h>

#include "comm_mngr.h"
#include "mocks/rfcomms_mock.h"
#include "mocks/ITimer_mock.h"
#include "mocks/IFlash_mock.h"
#include <pb_encode.h>
#include "gb_messages.pb.h"
#include "firmware/FWUpdate.h"

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::Ge;
using ::testing::NotNull;
using ::testing::SetArrayArgument;
using ::testing::DoAll;

class CommManagerTest: public ::testing::Test
{
protected:
    void SetUp() override
    {
        _fwupdate = new FWUpdate(_flash);
        _mngr = new CommManager(_uniqueID, _comms, _timer, 500, *_fwupdate);
    }

    void TearDown() override
    {
        delete _mngr;
        delete _fwupdate;
    }

    uint32_t _uniqueID = 1234;
    MockRFComms _comms;
    ITimer_mock _timer;
    IFlash_mock _flash;
    FWUpdate *_fwupdate;

    CommManager *_mngr;
};

TEST_F(CommManagerTest, SendHeartBeat)
{
    // Send data back to device
    LoraMsg2 msg = LoraMsg2_init_default;

    msg.status = Status_HEARTBEAT;

    uint8_t pbbuf[250];

    pb_ostream_t stream = pb_ostream_from_buffer(pbbuf, sizeof(pbbuf));
    bool status = pb_encode(&stream, LoraMsg2_fields, &msg);
    EXPECT_TRUE(status);

    // TODO: These are a bit rigid
    EXPECT_CALL(_comms, sendData(_, Ge(22)))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(22));

    EXPECT_CALL(_comms, getData(NotNull(), Ge(150), Ge(250)))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArrayArgument<0>(pbbuf, pbbuf + stream.bytes_written), Return(stream.bytes_written)));

    int txSize = _mngr->sendData(1000, 25, 3800, 1500, 5, -60);
    EXPECT_GE(txSize, 10);

    // comm_mngr will send data, wait a timeout value and then read data
    int processedData = _mngr->processResponse();
    EXPECT_EQ(0, processedData);
    EXPECT_EQ(_mngr->getLastStatus(), Status_HEARTBEAT);
}

TEST_F(CommManagerTest, SendImpact)
{
    std::vector<int16_t> imu;
    // Populdate the IMU with 10 samples
    for (int i = 0; i < 10; i++)
    {
        //accel
        imu.push_back(1);       // x
        imu.push_back(2);       // y
        imu.push_back(1000);    // z
        // gyro
        imu.push_back(3);       // x
        imu.push_back(4);       // y
        imu.push_back(5);       // z
    }

    // Send data back to device
    LoraMsg2 msg = LoraMsg2_init_default;

    msg.status = Status_IMPACT;

    uint8_t pbbuf[250];

    pb_ostream_t stream = pb_ostream_from_buffer(pbbuf, sizeof(pbbuf));
    pb_encode(&stream, LoraMsg2_fields, &msg);

    // TODO: These are a bit rigid
    EXPECT_CALL(_comms, sendData(_, Ge(100)))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(145));

    EXPECT_CALL(_comms, getData(NotNull(), Ge(150), Ge(250)))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArrayArgument<0>(pbbuf, pbbuf + stream.bytes_written), Return(stream.bytes_written)));

    int txSize = _mngr->sendData(1000, 25, 3800, 1500, 5, -60, imu);
    EXPECT_GE(txSize, 100);

    // comm_mngr will send data, wait a timeout value and then read data
    int processedData = _mngr->processResponse();
    EXPECT_EQ(0, processedData);
    EXPECT_EQ(_mngr->getLastStatus(), Status_IMPACT);
}

TEST_F(CommManagerTest, SendValidProgrammingPacketsWithNoErrors)
{

}

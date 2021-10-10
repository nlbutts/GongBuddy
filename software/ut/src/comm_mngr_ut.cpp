#include <stdint.h>
#include <gtest/gtest.h>

#include "comm_mngr.h"
#include "mocks/rfcomms_mock.h"
#include "mocks/ITimer_mock.h"
#include "mocks/IFlash_mock.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "gb_messages.pb.h"
#include "firmware/FWUpdate.h"
#include "firmware/ImageHeader.h"
#include <cstdio>

using ::testing::AtLeast;
using ::testing::Return;
using ::testing::_;
using ::testing::Ge;
using ::testing::NotNull;
using ::testing::SetArrayArgument;
using ::testing::DoAll;
using ::testing::Invoke;

class CommManagerTest: public ::testing::Test
{
protected:
    void SetUp() override
    {
        _fwupdate = new FWUpdate(_flash);
        _mngr = new CommManager(_uniqueID, _comms, _timer, 500, *_fwupdate);
        CommManagerTest::_baseAddress = 0xFFFFFFFF;
    }

    void TearDown() override
    {
        delete _mngr;
        delete _fwupdate;
    }

    uint32_t loadfw(std::string filename, std::vector<uint8_t> &data)
    {
        FILE * f = fopen(filename.c_str(), "rb");
        if (f != nullptr)
        {
            fseek(f, 0, SEEK_END);
            uint32_t size = ftell(f);
            fseek(f, 0, SEEK_SET);
            uint8_t * ptr = (uint8_t*)malloc(size);
            auto readbytes = fread(ptr, 1, size, f);
            fclose(f);
            std::vector<uint8_t> temp(ptr, ptr + readbytes);
            free(ptr);
            data = temp;
        }

        return data.size();
    }

    static int dataFromGB(uint8_t * pbData, int length)
    {
        CommManagerTest::_data.resize(length);
        for (int i = 0; i < length; i++)
        {
            _data[i] = pbData[i];
        }
        return length;
    }

    static bool flashWrite(uint32_t address, uint64_t data)
    {
        if (_baseAddress == 0xFFFFFFFF)
        {
            _baseAddress = address;
        }
        uint32_t offset = address - CommManagerTest::_baseAddress;
        // if (CommManagerTest::_flashData.capacity() < (offset + 8))
        // {
        //     CommManagerTest::_flashData.reserve(CommManagerTest::_flashData.size() + 256);
        // }

        CommManagerTest::_flashData[offset    ] = ((data      ) & 0xFF);
        CommManagerTest::_flashData[offset + 1] = ((data >>  8) & 0xFF);
        CommManagerTest::_flashData[offset + 2] = ((data >> 16) & 0xFF);
        CommManagerTest::_flashData[offset + 3] = ((data >> 24) & 0xFF);
        CommManagerTest::_flashData[offset + 4] = ((data >> 32) & 0xFF);
        CommManagerTest::_flashData[offset + 5] = ((data >> 40) & 0xFF);
        CommManagerTest::_flashData[offset + 6] = ((data >> 48) & 0xFF);
        CommManagerTest::_flashData[offset + 7] = ((data >> 56) & 0xFF);
        return true;
    }

    static uint8_t flashRead(uint32_t address)
    {
        uint32_t offset = address - CommManagerTest::_baseAddress;
        if (offset < CommManagerTest::MaxBufferSize)
        {
            return CommManagerTest::_flashData[offset];
        }
        return 0;
    }

    static void dumpFlash()
    {
        FILE * f = fopen("dump.bin", "wb");
        if (f != nullptr)
        {
            fwrite(_flashData, 1, MaxBufferSize, f);
            fclose(f);
        }
    }

    uint32_t _uniqueID = 1234;
    MockRFComms _comms;
    ITimer_mock _timer;
    IFlash_mock _flash;
    FWUpdate *_fwupdate;

    CommManager *_mngr;
    static std::vector<uint8_t> _data;
    //static std::vector<uint8_t> _flashData;
    static constexpr uint32_t MaxBufferSize = 0x100000;
    static uint8_t _flashData[CommManagerTest::MaxBufferSize];
    static uint32_t _flashDataSize;
    static uint32_t _baseAddress;
};

std::vector<uint8_t> CommManagerTest::_data;
//std::vector<uint8_t> CommManagerTest::_flashData;
uint8_t CommManagerTest::_flashData[0x100000] = {0xFF};
uint32_t CommManagerTest::_flashDataSize = 0;
uint32_t CommManagerTest::_baseAddress;

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
    constexpr int PacketSize = 200;

    // Send data back to device
    LoraMsg2 msg = LoraMsg2_init_default;

    std::vector<uint8_t> fwblob;
    loadfw("app.image", fwblob);

    auto packets = fwblob.size() / PacketSize;
    if ((fwblob.size() % PacketSize) != 0)
    {
        packets++;
    }

    Crc32_Normal fwcrc;
    fwcrc.update(fwblob.data(), fwblob.size());

    msg.status = Status_REPROGRAMMING;
    msg.fw_setup.bytes_per_packet = PacketSize;
    msg.fw_setup.start_address = 0;
    msg.fw_setup.total_packets = packets;
    msg.fw_setup.fw_image_size = fwblob.size();
    msg.fw_setup.fw_crc = fwcrc.getCrc();
    msg.has_fw_setup = true;

    uint8_t pbbuf[250];

    pb_ostream_t stream = pb_ostream_from_buffer(pbbuf, sizeof(pbbuf));
    bool status = pb_encode(&stream, LoraMsg2_fields, &msg);
    EXPECT_TRUE(status);

    EXPECT_CALL(_comms, sendData(_, _))
        .Times(AtLeast(1))
        .WillRepeatedly(Invoke(CommManagerTest::dataFromGB));

    EXPECT_CALL(_comms, getData(NotNull(), Ge(150), Ge(250)))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArrayArgument<0>(pbbuf, pbbuf + stream.bytes_written), Return(stream.bytes_written)));

    EXPECT_CALL(_flash, write(_, _))
        .Times(AtLeast(1))
        .WillRepeatedly(Invoke(CommManagerTest::flashWrite));

    EXPECT_CALL(_flash, read(_))
        .Times(AtLeast(1))
        .WillRepeatedly(Invoke(CommManagerTest::flashRead));

    EXPECT_CALL(_timer, setTimerMs(_))
        .Times(AtLeast(1));

    int txSize = _mngr->sendData(1000, 25, 3800, 1500, 5, -60);
    EXPECT_GE(txSize, 10);

    // comm_mngr will send data, wait a timeout value and then read data
    int processedData = _mngr->processResponse();
    EXPECT_EQ(1, processedData);
    EXPECT_EQ(_mngr->getLastStatus(), Status_REPROGRAMMING);

    {
        pb_istream_t istream = pb_istream_from_buffer(_data.data(), _data.size());
        bool status = pb_decode(&istream, LoraMsg2_fields, &msg);
        EXPECT_TRUE(status);
    }
    EXPECT_EQ(msg.status, Status_REPROGRAMMING);
    EXPECT_EQ(msg.fw_status.status, FWUpdateStatus_FWStatus_READY_FOR_PAYLOAD);

    for (uint32_t i = 0; i < packets; i++)
    {
        msg = LoraMsg2_init_default;
        msg.status = Status_REPROGRAMMING;
        msg.has_reprog = true;
        msg.reprog.packet = i;
        int bytesRemaining = fwblob.size() - (i * PacketSize);
        int bytesToCopy = bytesRemaining > PacketSize ? PacketSize : bytesRemaining;
        for (int j = 0; j < bytesToCopy; j++)
        {
            msg.reprog.data.bytes[j] = fwblob[i*PacketSize + j];
        }
        msg.reprog.data.size = bytesToCopy;
        pb_ostream_t temp = pb_ostream_from_buffer(pbbuf, sizeof(pbbuf));
        stream = temp;
        status = pb_encode(&stream, LoraMsg2_fields, &msg);
        EXPECT_TRUE(status);

        EXPECT_CALL(_comms, getData(NotNull(), Ge(150), Ge(250)))
            .Times(AtLeast(1))
            .WillRepeatedly(DoAll(SetArrayArgument<0>(pbbuf, pbbuf + stream.bytes_written), Return(stream.bytes_written)));

        processedData = _mngr->processResponse();
        EXPECT_EQ(1, processedData);
        EXPECT_EQ(_mngr->getLastStatus(), Status_REPROGRAMMING);
    }

    {
        pb_istream_t istream = pb_istream_from_buffer(_data.data(), _data.size());
        bool status = pb_decode(&istream, LoraMsg2_fields, &msg);
        EXPECT_TRUE(status);
    }
    EXPECT_EQ(msg.status, Status_REPROGRAMMING);
    EXPECT_EQ(msg.fw_status.status, FWUpdateStatus_FWStatus_VALID_FW_BLOB);
    dumpFlash();
}

TEST_F(CommManagerTest, SendValidProgrammingPacketsWithBadData)
{
    constexpr int PacketSize = 200;

    // Send data back to device
    LoraMsg2 msg = LoraMsg2_init_default;

    std::vector<uint8_t> fwblob;
    loadfw("app.image", fwblob);

    fwblob[100] = 0x42;

    auto packets = fwblob.size() / PacketSize;
    if ((fwblob.size() % PacketSize) != 0)
    {
        packets++;
    }

    Crc32_Normal fwcrc;
    fwcrc.update(fwblob.data(), fwblob.size());

    msg.status = Status_REPROGRAMMING;
    msg.fw_setup.bytes_per_packet = PacketSize;
    msg.fw_setup.start_address = 0;
    msg.fw_setup.total_packets = packets;
    msg.fw_setup.fw_image_size = fwblob.size();
    msg.fw_setup.fw_crc = fwcrc.getCrc();
    msg.has_fw_setup = true;

    uint8_t pbbuf[250];

    pb_ostream_t stream = pb_ostream_from_buffer(pbbuf, sizeof(pbbuf));
    bool status = pb_encode(&stream, LoraMsg2_fields, &msg);
    EXPECT_TRUE(status);

    EXPECT_CALL(_comms, sendData(_, _))
        .Times(AtLeast(1))
        .WillRepeatedly(Invoke(CommManagerTest::dataFromGB));

    EXPECT_CALL(_comms, getData(NotNull(), Ge(150), Ge(250)))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArrayArgument<0>(pbbuf, pbbuf + stream.bytes_written), Return(stream.bytes_written)));

    EXPECT_CALL(_flash, write(_, _))
        .Times(AtLeast(1))
        .WillRepeatedly(Invoke(CommManagerTest::flashWrite));

    EXPECT_CALL(_flash, read(_))
        .Times(AtLeast(1))
        .WillRepeatedly(Invoke(CommManagerTest::flashRead));

    EXPECT_CALL(_timer, setTimerMs(_))
        .Times(AtLeast(1));

    int txSize = _mngr->sendData(1000, 25, 3800, 1500, 5, -60);
    EXPECT_GE(txSize, 10);

    // comm_mngr will send data, wait a timeout value and then read data
    int processedData = _mngr->processResponse();
    EXPECT_EQ(1, processedData);
    EXPECT_EQ(_mngr->getLastStatus(), Status_REPROGRAMMING);

    {
        pb_istream_t istream = pb_istream_from_buffer(_data.data(), _data.size());
        bool status = pb_decode(&istream, LoraMsg2_fields, &msg);
        EXPECT_TRUE(status);
    }
    EXPECT_EQ(msg.status, Status_REPROGRAMMING);
    EXPECT_EQ(msg.fw_status.status, FWUpdateStatus_FWStatus_READY_FOR_PAYLOAD);

    for (uint32_t i = 0; i < packets; i++)
    {
        msg = LoraMsg2_init_default;
        msg.status = Status_REPROGRAMMING;
        msg.has_reprog = true;
        msg.reprog.packet = i;
        int bytesRemaining = fwblob.size() - (i * PacketSize);
        int bytesToCopy = bytesRemaining > PacketSize ? PacketSize : bytesRemaining;
        for (int j = 0; j < bytesToCopy; j++)
        {
            msg.reprog.data.bytes[j] = fwblob[i*PacketSize + j];
        }
        msg.reprog.data.size = bytesToCopy;
        pb_ostream_t temp = pb_ostream_from_buffer(pbbuf, sizeof(pbbuf));
        stream = temp;
        status = pb_encode(&stream, LoraMsg2_fields, &msg);
        EXPECT_TRUE(status);

        EXPECT_CALL(_comms, getData(NotNull(), Ge(150), Ge(250)))
            .Times(AtLeast(1))
            .WillRepeatedly(DoAll(SetArrayArgument<0>(pbbuf, pbbuf + stream.bytes_written), Return(stream.bytes_written)));

        processedData = _mngr->processResponse();
        EXPECT_EQ(1, processedData);
        EXPECT_EQ(_mngr->getLastStatus(), Status_REPROGRAMMING);
    }

    {
        pb_istream_t istream = pb_istream_from_buffer(_data.data(), _data.size());
        bool status = pb_decode(&istream, LoraMsg2_fields, &msg);
        EXPECT_TRUE(status);
    }
    EXPECT_EQ(msg.status, Status_REPROGRAMMING);
    EXPECT_EQ(msg.fw_status.status, FWUpdateStatus_FWStatus_VALID_FW_BLOB);
    dumpFlash();
}


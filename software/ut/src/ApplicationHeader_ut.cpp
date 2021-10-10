#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <gtest/gtest.h>

#include <firmware/ApplicationHeader.h>
using Firmware::ApplicationHeader;

class ApplicationHeaderTest : public ::testing::Test
{
public:
    ApplicationHeaderTest()
    {
        appImage = new uint8_t[APP_SPACE_SIZE];
        memset(appImage, 0, APP_SPACE_SIZE);

        // TODO: Copy in a valid copy of the header
        static const uint8_t HEADER[] = {0x03, 0x21, 0x9c, 0x15, 0x0A, 0x00, 0x00, 0x00};
        memcpy(&appImage[0], HEADER, sizeof(HEADER));

        // Copy in a valid payload
        static const uint8_t PAYLOD[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00};
        memcpy(&appImage[HEADER_LENGTH], PAYLOD, sizeof(PAYLOD));
    }

    ~ApplicationHeaderTest()
    {
        delete[] appImage;
    }

    virtual void SetUp()
    {
    }

    virtual void TearDown()
    {
    }

public:
    static const uint32_t HEADER_LENGTH     = 512;
    static const uint32_t PAYLOAD_LENGTH    = 10;
    static const uint32_t APP_SPACE_SIZE    = HEADER_LENGTH + PAYLOAD_LENGTH;
    static const uint32_t APP_CRC           = 0x159C2103;

    uint8_t*        appImage;
};

TEST_F(ApplicationHeaderTest, TestDeserialization)
{
    static const uint32_t   EXPECTED_APPLICATION_SIZE   = PAYLOAD_LENGTH;
    static const uint32_t   EXPECTED_CRC                = APP_CRC;
    ApplicationHeader dut(appImage, APP_SPACE_SIZE);

    EXPECT_EQ(EXPECTED_APPLICATION_SIZE, dut.getApplicationSize());
    EXPECT_EQ(EXPECTED_CRC, dut.getCrc());
    EXPECT_TRUE(dut.isApplicationValid());
}


TEST_F(ApplicationHeaderTest, TestInvalidApp1)
{
    ApplicationHeader dut(appImage, APP_SPACE_SIZE);

    // Flip a bit in the header, byte 9 is at the end of the length field
    appImage[9] = 0x01;

    EXPECT_FALSE(dut.isApplicationValid());
}

TEST_F(ApplicationHeaderTest, TestInvalidApp2)
{
    ApplicationHeader dut(appImage, APP_SPACE_SIZE);

    // Change the fist bytes in the payload
    appImage[HEADER_LENGTH] = 0xFF;

    EXPECT_FALSE(dut.isApplicationValid());
}


TEST_F(ApplicationHeaderTest, TestMaxAppLength)
{
    // The max application size is passed as 1 less than the actual
    // fake image created for this test
    ApplicationHeader dut(appImage, APP_SPACE_SIZE - 1);
    EXPECT_FALSE(dut.isApplicationValid());
}


TEST_F(ApplicationHeaderTest, TestAlignment)
{
    EXPECT_EQ(0, ApplicationHeader::getApplicationStartAddress(0) % 512);
}

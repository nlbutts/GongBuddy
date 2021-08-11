#include <stdint.h>
#include <gtest/gtest.h>

#include <iostream>

#include <crc/Crc32.h>

#include <firmware/ImageHeader.h>
using Firmware::ImageHeader;

TEST(ImageHeaderTest, SerializationTest)
{
    static const ImageHeader::Type EXPECTED_TYPE = ImageHeader::APPLICATION;
    static const uint32_t EXPECTED_HW_COMPAT = 0xAABBCCDD;
    static const uint8_t PAYLOAD[] = {0x01, 0x02, 0x03, 0x04};
    static const uint32_t EXPECTED_PAYLOAD_LENGTH = sizeof(PAYLOAD);

    static const uint32_t EXPECTED_CRC = 0x439FA7A8;
    static const uint16_t EXPECTED_CHECKSUM = 0x054E;
    static const uint8_t EXPECTED_SERIALIZATION[ImageHeader::IMAGE_HEADER_LENGTH] =
            {0x4E,0x05,0xA8,0xA7,0x9F,0x43,0x04,0x00,
             0x00,0x00,0x01,0xDD,0xCC,0xBB,0xAA,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    ASSERT_EQ(32, sizeof(EXPECTED_SERIALIZATION));

    ImageHeader dut(EXPECTED_TYPE,
                    EXPECTED_HW_COMPAT,
                    PAYLOAD,
                    EXPECTED_PAYLOAD_LENGTH);

    EXPECT_TRUE(dut.isImageValid());
    EXPECT_EQ(EXPECTED_TYPE,            dut.getImageType());
    EXPECT_EQ(EXPECTED_HW_COMPAT,       dut.getHwCompatField());
    EXPECT_EQ(EXPECTED_PAYLOAD_LENGTH,  dut.getPayloadLength());
    EXPECT_EQ(EXPECTED_CRC,             dut.getCrc());
    EXPECT_EQ(EXPECTED_CHECKSUM,        dut.getChecksum());

    uint8_t serialization[ImageHeader::IMAGE_HEADER_LENGTH];
    dut.serializeHeader(serialization);
    for(uint32_t i = 0; i < sizeof(EXPECTED_SERIALIZATION); ++i)
    {
        /*std::cout << "i = " << i
                  << "\texpected = " << std::hex << static_cast<unsigned int>(EXPECTED_SERIALIZATION[i])
                  << "\tactual = " << std::hex << static_cast<unsigned int>(serialization[i])
                  << std::endl;*/

        EXPECT_EQ(static_cast<unsigned int>(EXPECTED_SERIALIZATION[i]),
                  static_cast<unsigned int>(serialization[i]));

    }
}


TEST(ImageHeaderTest, DeserializationTest)
{
    static const ImageHeader::Type EXPECTED_TYPE = ImageHeader::APPLICATION;
    static const uint32_t EXPECTED_HW_COMPAT = 0xAABBCCDD;
    static const uint8_t PAYLOAD[] = {0x01, 0x02, 0x03, 0x04};
    static const uint32_t EXPECTED_PAYLOAD_LENGTH = sizeof(PAYLOAD);
    static const uint32_t EXPECTED_CRC = 0x439FA7A8;
    static const uint16_t EXPECTED_CHECKSUM = 0x054E;
    static const uint8_t SERIALIZED_IMAGE[] =
            {0x4E,0x05,0xA8,0xA7,0x9F,0x43,0x04,0x00,
             0x00,0x00,0x01,0xDD,0xCC,0xBB,0xAA,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x01,0x02,0x03,0x04};

    ASSERT_EQ(36, sizeof(SERIALIZED_IMAGE));

    ImageHeader dut(SERIALIZED_IMAGE, 4);

    EXPECT_TRUE(dut.isImageValid());
    EXPECT_EQ(EXPECTED_CRC,             dut.getCrc());
    EXPECT_EQ(EXPECTED_CHECKSUM,        dut.getChecksum());
    EXPECT_EQ(EXPECTED_PAYLOAD_LENGTH,  dut.getPayloadLength());
    EXPECT_EQ(EXPECTED_TYPE,            dut.getImageType());
    EXPECT_EQ(EXPECTED_HW_COMPAT,       dut.getHwCompatField());
    EXPECT_EQ(&SERIALIZED_IMAGE[32],    dut.getPayloadPointer());
}


TEST(ImageHeaderTest, Constants)
{
    EXPECT_EQ(static_cast<uint32_t>(ImageHeader::APPLICATION),      1);
    EXPECT_EQ(static_cast<uint32_t>(ImageHeader::APPLICATION_LZMA), 2);
    EXPECT_EQ(static_cast<uint32_t>(ImageHeader::BOOTLOADER),       3);
    EXPECT_EQ(static_cast<uint32_t>(ImageHeader::BOOTLOADER_LZMA),  4);
}


TEST(ImageHeaderTest, BadImage)
{
    static const uint8_t SERIALIZED_IMAGE[] =
            {0x4E,0x05,0xA8,0xA7,0x9F,0x43,0x04,0x01, /* Changed from 0 to 1*/
             0x00,0x00,0x01,0xDD,0xCC,0xBB,0xAA,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x01,0x02,0x03,0x04};

    ASSERT_EQ(36, sizeof(SERIALIZED_IMAGE));

    ImageHeader dut(SERIALIZED_IMAGE, 4);

    EXPECT_FALSE(dut.isImageValid());
}

TEST(ImageHeaderTest, MaxImageLength)
{
    static const uint8_t SERIALIZED_IMAGE[] =
            {0x4E,0x05,0xA8,0xA7,0x9F,0x43,0x04,0x00,
             0x00,0x00,0x01,0xDD,0xCC,0xBB,0xAA,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
             0x01,0x02,0x03,0x04};

    ASSERT_EQ(36, sizeof(SERIALIZED_IMAGE));

    ImageHeader dut(SERIALIZED_IMAGE, 3);

    // Image should not be valid because the length field is longer
    // than the maximum allowed payload size
    EXPECT_FALSE(dut.isImageValid());
}

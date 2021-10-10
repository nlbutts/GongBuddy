#include <stdint.h>
#include <gtest/gtest.h>

#include <Crc32.h>

TEST(Crc32_NormalTest, TestInitialValue)
{
    // Expected value generated with srec_cat
    static const uint32_t EXPECTED_CRC = 0x00000000;

    Crc32_Normal crc;
    crc.update(nullptr, 0);
    uint32_t calccrc = crc.getCrc();

    EXPECT_EQ(EXPECTED_CRC, calccrc);
}

TEST(Crc32_NormalTest, TestBlock)
{
    // Expected value generated with srec_cat
    static const uint8_t DATA[] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    static const uint32_t EXPECTED_CRC = 0xEA8C89C0;

    ASSERT_EQ(8, sizeof(DATA));
    Crc32_Normal crc;
    crc.update(DATA, 8);
    uint32_t calccrc = crc.getCrc();

    EXPECT_EQ(EXPECTED_CRC, calccrc);
}

TEST(Crc32_NormalTest, TestBlock2)
{
    // Expected value generated with srec_cat
    static const uint8_t DATA[] = {0xAB, 0xCD, 0xEF, 0x01, 0x00, 0xFF, 0x20, 0x96, 0xC5, 0xD1, 0xDF, 0xAA, 0x55};
    static const uint32_t EXPECTED_CRC = 0x3A458204;

    ASSERT_EQ(13, sizeof(DATA));
    Crc32_Normal crc;
    crc.update(DATA, 13);
    uint32_t calccrc = crc.getCrc();

    EXPECT_EQ(EXPECTED_CRC, calccrc);
}

TEST(Crc32_NormalTest, TestPartialBlock)
{
    // Expected value generated with srec_cat
    static const uint8_t DATA[] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    static const uint32_t EXPECTED_CRC = 0xEA8C89C0;

    ASSERT_EQ(8, sizeof(DATA));
    Crc32_Normal crc;
    crc.update(&DATA[0], 4);
    crc.update(&DATA[4], 4);
    uint32_t calccrc = crc.getCrc();

    EXPECT_EQ(EXPECTED_CRC, calccrc);
}

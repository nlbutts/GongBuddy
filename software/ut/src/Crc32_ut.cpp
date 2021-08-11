#include <stdint.h>
#include <gtest/gtest.h>

#include <crc/Crc32.h>

TEST(Crc32_NormalTest, TestInitialValue)
{
    // Expected value generated with srec_cat
    static const uint32_t EXPECTED_CRC = 0x00000000;

    uint32_t crc = crc_calculate(nullptr, 0);

    EXPECT_EQ(EXPECTED_CRC, crc);
}

TEST(Crc32_NormalTest, TestBlock)
{
    // Expected value generated with srec_cat
    static const uint8_t DATA[] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    static const uint32_t EXPECTED_CRC = 0xEA8C89C0;

    ASSERT_EQ(8, sizeof(DATA));
    uint32_t crc = crc_calculate(DATA, 8);

    EXPECT_EQ(EXPECTED_CRC, crc);
}

TEST(Crc32_NormalTest, TestBlock2)
{
    // Expected value generated with srec_cat
    static const uint8_t DATA[] = {0xAB, 0xCD, 0xEF, 0x01, 0x00, 0xFF, 0x20, 0x96, 0xC5, 0xD1, 0xDF, 0xAA, 0x55};
    static const uint32_t EXPECTED_CRC = 0x3A458204;

    ASSERT_EQ(13, sizeof(DATA));
    uint32_t crc = crc_calculate(DATA, 13);

    EXPECT_EQ(EXPECTED_CRC, crc);
}

TEST(Crc32_NormalTest, TestPartialBlock)
{
    // Expected value generated with srec_cat
    static const uint8_t DATA[] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    static const uint32_t EXPECTED_CRC = 0xEA8C89C0;

    ASSERT_EQ(8, sizeof(DATA));
    uint32_t crc = 0xFFFFFFFF;
    crc = crc_partial_calculate(crc, &DATA[0], 4);
    crc = crc_partial_calculate(crc, &DATA[4], 4);
    crc = crc_partial_complete(crc);

    EXPECT_EQ(EXPECTED_CRC, crc);
}

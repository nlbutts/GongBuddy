#ifndef CRC_CRC32_H
#define CRC_CRC32_H

#include <stdint.h>

/**
 * @brief Updates the CRC with new data
 * @details This may be called multiple times as data becomes available,
 *          the CRC does not need to be calculated in a single operation
 *
 * @param crc the existing CRC or zero to start a new CRC
 * @param data Pointer to the data to add in
 * @param len Length of the data to add in
 * @return uint32_t the resulting CRC
 */
uint32_t crc_calculate(uint32_t crc, const uint8_t* data, uint32_t len);

#endif /* CRC_CRC32_H */
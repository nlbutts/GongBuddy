#ifndef CRC_CRC32_H
#define CRC_CRC32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculates the CRC on a block of data
 * @details This version of the function can be called to calculate the CRC
 * on a block of data. If you need to do multiple, discontinuous blocks, call
 * crc_partial_calculate
 *
 * @param data Pointer to the data to add in
 * @param len Length of the data to add in
 * @return uint32_t the resulting CRC
 */
uint32_t crc_calculate(const uint8_t* data, uint32_t len);

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
uint32_t crc_partial_calculate(uint32_t crc, const uint8_t* data, uint32_t len);

/**
 * @brief Completes the partial CRC calculations
 * @details Call this function at the end of the disconintous block of
 * crc_partial_calculate calls.
 *
 * @param crc the existing CRC or zero to start a new CRC
 * @return uint32_t the resulting CRC
 */
uint32_t crc_partial_complete(uint32_t crc);

#ifdef __cplusplus
}
#endif

#endif /* CRC_CRC32_H */
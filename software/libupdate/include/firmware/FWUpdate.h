#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_

#include <stdint.h>

/**
 * @brief Constructor
 *
 * @param appSpace Pointer to the start of application space
 * @param appSpaceLength Size of the application flash space. If the length field
 *        within the application header is higher than this, the CRC will not be
 *        executed and the image will not be valid.
 */
void FWUpdate_execute(const uint8_t* appSpace, uint32_t appSpaceLength);

#endif /* FW_UPDATE_H_ */
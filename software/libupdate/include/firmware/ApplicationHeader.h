#ifndef FIRMWARE_APPLICATION_HEADER_H
#define FIRMWARE_APPLICATION_HEADER_H

#include <stdint.h>

namespace Firmware {


/**
 * @brief Class for validation the application
 */
class ApplicationHeader
{
public:
    /**
     * @brief Constructor
     *
     * @param appSpace Pointer to the start of application space
     * @param appSpaceLength Size of the application flash space. If the length field
     *        within the application header is higher than this, the CRC will not be
     *        executed and the image will not be valid.
     */
    ApplicationHeader(const uint8_t* appSpace, uint32_t appSpaceLength);

    /**
     * @brief Destructor
     */
    virtual ~ApplicationHeader();

    /**
     * @brief Gets the CRC from the application space
     * @details The returned value in not guaranteed to be correct if
     *          isApplicationValdi() does not return true.
     *
     * @return CRC-32 field value
     */
    uint32_t getCrc(void) const;

    /**
     * @brief Gets the size of the application, not including header information
     * @details The returned value in not guaranteed to be correct if
     *          isApplicationValdi() does not return true.
     *
     * @return Length in bytes of the application
     */
    uint32_t getApplicationSize(void) const;

    /**
     * @brief Checks if the application is valid
     * @details [long description]
     * @return [description]
     */
    bool isApplicationValid() const;

    /**
     * @brief Gets the application start address
     *
     * @param appSpaceAddress Starting address of application space
     * @return Address to the application IVT
     */
    static uint32_t getApplicationStartAddress(uint32_t appSpaceAddress);

private:
    static const uint32_t CRC_OFFSET = 0;
    static const uint32_t LENGTH_OFFSET = 4;
    static const uint32_t APPLICATION_START_OFFSET = 512;

    static const uint32_t HEADER_CRC_START_OFFSET = LENGTH_OFFSET;
    static const uint32_t HEADER_BYTES_TO_CRC = APPLICATION_START_OFFSET - HEADER_CRC_START_OFFSET;

private:
    const uint8_t*      _appSpace;
    uint32_t            _appSpaceLength;
};

} // namespace Firmware

#endif /* FIRMWARE_APPLICATION_HEADER_H */
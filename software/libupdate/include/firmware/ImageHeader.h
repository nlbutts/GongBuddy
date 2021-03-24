#ifndef FIRMWARE_IMAGE_HEADER_H
#define FIRMWARE_IMAGE_HEADER_H

#include <stdint.h>

namespace Firmware {


/**
 * @brief Update image serializer / deserializer
 * See confluence for information about the update process
 * http://development-doc:8015/display/FRON/Firmware+Update+Architecture
 */
class ImageHeader
{
public:
    /**
     * @brief Image header length in bytes
     */
    static const uint32_t IMAGE_HEADER_LENGTH = 32;

    enum Type
    {
        INVALID             = 0,
        APPLICATION         = 1,
        APPLICATION_LZMA    = 2,
        BOOTLOADER          = 3,
        BOOTLOADER_LZMA     = 4
    };

public:
    /**
     * @brief Constructor for creating update image headers
     * @details This constructor is used by the imagegen utility to create
     *          a firmware update image header
     *
     * @param imageType Type of image to create
     * @param HwCompatField Hardware compatibility field
     * @param payload Update image payload
     * @param payloadLength Length of the image payload
     */
    ImageHeader(Type imageType, uint32_t HwCompatField,
                const uint8_t* payload, uint32_t payloadLength);

    /**
     * @brief Constructor for deserializing image header and image information
     *
     * @param image Pointer to the image to deserialize
     * @param maxPayloadLength Maximum size of a firmware image. If the length field
     *        within the image is higher than this, the CRC will not be executed
     *        and the image will not be valid.
     */
    ImageHeader(const uint8_t* image, uint32_t maxPayloadLength);

    /**
     * @brief Destructor
     */
    virtual ~ImageHeader();

    /**
     * @brief Serialize (pack) header information into a buffer
     * This function is intended to be used with the serializing constructor
     *
     * @param buffer Buffer to serialize header information into
     */
    void serializeHeader(uint8_t (&buffer)[IMAGE_HEADER_LENGTH]) const;

    /**
     * @brief Gets a pointer to the beginning of the update image payload
     * @details This function is intended to be used with the deserializing
     *          constructor
     * @return Pointer to the beginning of the image payload
     */
    const uint8_t* getPayloadPointer(void) const;

    /**
     * @brief Gets the length of the update image payload
     * @details This function is intended to be used with the deserializing
     *          constructor.
     *
     * @return Length of the image payload
     */
    uint32_t getPayloadLength(void) const;

    /**
     * @brief Checks if the update image is valid
     *
     * @return true if the image is valid, else false
     */
    bool isImageValid(void) const;

    /**
     * @brief Gets the type of the image header
     * @details This function is intended to be used with the deserializing
     *          constructor.
     * @return Type
     */
    Type getImageType(void) const;

    /**
     * @brief Gets the hardware compatibility field out of the image header
     * @details This function is intended to be used with the deserializing
     *          constructor.
     *
     * @return Hardware compatibility field
     */
    uint32_t getHwCompatField(void) const;

    /**
     * @brief Gets the CRC from the image header
     *
     * @return CRC-32 field value
     */
    uint32_t getCrc(void) const;

    /**
     * @brief Gets the AGCO compatible checksum from the image header
     * @return Integer sum of the payload and header
     */
    uint16_t getChecksum(void) const;

    /**
     * @brief Checks if the image is of a bootblock type
     *
     * @return true if the image is for a bootblock
     */
    bool isBootblock(void) const;

    /**
     * @brief Checks if the image is a compressed type
     *
     * @return true if the image is compressed, else false
     */
    bool isCompressed(void) const;

private:
    static const uint32_t SUM16_OFFSET          = 0;
    static const uint32_t CRC_OFFSET            = 2;
    static const uint32_t LENGTH_OFFSET         = 6;
    static const uint32_t IMAGE_TYPE_OFFSET     = 10;
    static const uint32_t HW_COMPAT_OFFSET      = 11;

    // Length field is the first field to start CRC'ing
    static const uint32_t HEADER_CRC_START_OFFSET   = LENGTH_OFFSET;
    static const uint32_t HEADER_BYTES_TO_CRC       = IMAGE_HEADER_LENGTH - HEADER_CRC_START_OFFSET;

    uint8_t         _header[IMAGE_HEADER_LENGTH];
    const uint8_t*  _payload;
    const uint32_t  _maxPayloadLength;

private:
    static const uint8_t* getImagePayloadPointer(const uint8_t* image);
};

} // namespace Firmware

#endif /* FIRMWARE_IMAGE_HEADER_H */
#include <stddef.h>
#include <string.h>

extern "C" {
#include <crc/Crc32.h>
}
#include <firmware/ImageHeader.h>

namespace Firmware {

ImageHeader::ImageHeader(Type imageType, uint32_t HwCompatField,
                         const uint8_t* payload, uint32_t payloadLength)
: _payload(payload)
, _maxPayloadLength(payloadLength)
{
    memset(_header, 0, IMAGE_HEADER_LENGTH);

    // Length field (little endian)
    _header[LENGTH_OFFSET    ] = static_cast<uint8_t>((payloadLength     )  & 0xFF);
    _header[LENGTH_OFFSET + 1] = static_cast<uint8_t>((payloadLength >> 8)  & 0xFF);
    _header[LENGTH_OFFSET + 2] = static_cast<uint8_t>((payloadLength >> 16) & 0xFF);
    _header[LENGTH_OFFSET + 3] = static_cast<uint8_t>((payloadLength >> 24) & 0xFF);

    // Image type
    _header[IMAGE_TYPE_OFFSET] = static_cast<uint8_t>(imageType);

    // Hardware compatibility field
    _header[HW_COMPAT_OFFSET    ] = static_cast<uint8_t>((HwCompatField     )  & 0xFF);
    _header[HW_COMPAT_OFFSET + 1] = static_cast<uint8_t>((HwCompatField >> 8)  & 0xFF);
    _header[HW_COMPAT_OFFSET + 2] = static_cast<uint8_t>((HwCompatField >> 16) & 0xFF);
    _header[HW_COMPAT_OFFSET + 3] = static_cast<uint8_t>((HwCompatField >> 24) & 0xFF);

    // CRC
    uint32_t crc = 0;
    crc = crc_calculate(&_header[HEADER_CRC_START_OFFSET], HEADER_BYTES_TO_CRC);
    crc = crc_calculate(payload, payloadLength);

    _header[CRC_OFFSET    ] = static_cast<uint8_t>((crc     )  & 0xFF);
    _header[CRC_OFFSET + 1] = static_cast<uint8_t>((crc >> 8)  & 0xFF);
    _header[CRC_OFFSET + 2] = static_cast<uint8_t>((crc >> 16) & 0xFF);
    _header[CRC_OFFSET + 3] = static_cast<uint8_t>((crc >> 24) & 0xFF);

    // AGCO Checksum
    uint32_t checksum = 0;
    for(uint32_t i = CRC_OFFSET; i < IMAGE_HEADER_LENGTH; ++i)
    {
        checksum += _header[i];
    }

    for(uint32_t i = 0; i < payloadLength; ++i)
    {
        checksum += _payload[i];
    }

    _header[SUM16_OFFSET    ] = static_cast<uint8_t>((checksum     ) & 0xFF);
    _header[SUM16_OFFSET + 1] = static_cast<uint8_t>((checksum >> 8) & 0xFF);
}


ImageHeader::ImageHeader(const uint8_t* image, uint32_t maxPayloadLength)
: _payload(getImagePayloadPointer(image))
, _maxPayloadLength(maxPayloadLength)
{
    memcpy(_header, image, IMAGE_HEADER_LENGTH);
}

ImageHeader::~ImageHeader()
{
    _payload = NULL;
}

void ImageHeader::serializeHeader(uint8_t (&buffer)[IMAGE_HEADER_LENGTH]) const
{
    // Header information should already be valid from construction,
    // only have to copy it out
    memcpy(buffer, _header, IMAGE_HEADER_LENGTH);
}

const uint8_t* ImageHeader::getPayloadPointer(void) const
{
    return _payload;
}

uint32_t ImageHeader::getPayloadLength(void) const
{
    uint32_t length =  (static_cast<uint32_t>(_header[LENGTH_OFFSET    ])     )
                     | (static_cast<uint32_t>(_header[LENGTH_OFFSET + 1]) << 8)
                     | (static_cast<uint32_t>(_header[LENGTH_OFFSET + 2]) << 16)
                     | (static_cast<uint32_t>(_header[LENGTH_OFFSET + 3]) << 24);

    return length;
}

bool ImageHeader::isImageValid(void) const
{
    uint32_t payloadLength = getPayloadLength();
    if(payloadLength > _maxPayloadLength)
    {
        // Can't trust the payload length field, don't run the CRC
        return false;
    }

    uint32_t crc = 0;
    crc = crc_calculate(&_header[HEADER_CRC_START_OFFSET], HEADER_BYTES_TO_CRC);
    crc = crc_calculate(_payload, payloadLength);

    uint32_t storedCRC = (static_cast<uint32_t>(_header[CRC_OFFSET    ])     )
                       | (static_cast<uint32_t>(_header[CRC_OFFSET + 1]) << 8)
                       | (static_cast<uint32_t>(_header[CRC_OFFSET + 2]) << 16)
                       | (static_cast<uint32_t>(_header[CRC_OFFSET + 3]) << 24);

    return storedCRC == crc;
}

ImageHeader::Type ImageHeader::getImageType(void) const
{
    Type type = INVALID;

    Type x = static_cast<Type>(_header[IMAGE_TYPE_OFFSET]);
    switch(x)
    {
        case APPLICATION:
        case APPLICATION_LZMA:
        case BOOTLOADER:
        case BOOTLOADER_LZMA:
            type = x;
            break;

        case INVALID:
        default:
            type = INVALID;
            break;
    }

    return type;
}

uint32_t ImageHeader::getHwCompatField(void) const
{
    uint32_t hwCompat =  (static_cast<uint32_t>(_header[HW_COMPAT_OFFSET    ])     )
                       | (static_cast<uint32_t>(_header[HW_COMPAT_OFFSET + 1]) << 8)
                       | (static_cast<uint32_t>(_header[HW_COMPAT_OFFSET + 2]) << 16)
                       | (static_cast<uint32_t>(_header[HW_COMPAT_OFFSET + 3]) << 24);

    return hwCompat;
}


uint32_t ImageHeader::getCrc(void) const
{
    uint32_t crc =  (static_cast<uint32_t>(_header[CRC_OFFSET    ])     )
                  | (static_cast<uint32_t>(_header[CRC_OFFSET + 1]) << 8)
                  | (static_cast<uint32_t>(_header[CRC_OFFSET + 2]) << 16)
                  | (static_cast<uint32_t>(_header[CRC_OFFSET + 3]) << 24);

    return crc;
}


uint16_t ImageHeader::getChecksum(void) const
{
    uint16_t checksum =  (static_cast<uint16_t>(_header[SUM16_OFFSET    ])     )
                       | (static_cast<uint16_t>(_header[SUM16_OFFSET + 1]) << 8);

    return checksum;
}


const uint8_t* ImageHeader::getImagePayloadPointer(const uint8_t* image)
{
    return &image[IMAGE_HEADER_LENGTH];
}

bool ImageHeader::isBootblock(void) const
{
    Type i = getImageType();
    return (i == BOOTLOADER) || (i == BOOTLOADER_LZMA);
}

bool ImageHeader::isCompressed(void) const
{
    Type i = getImageType();
    return (i == BOOTLOADER_LZMA) || (i == APPLICATION_LZMA);
}

} // namespace Firmware

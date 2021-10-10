#include <stddef.h>
#include <firmware/ApplicationHeader.h>
#include <Crc32.h>

namespace Firmware {

ApplicationHeader::ApplicationHeader(const uint8_t* appSpace,
                                     uint32_t appSpaceLength)
: _appSpace(appSpace)
, _appSpaceLength(appSpaceLength)
{
}

ApplicationHeader::~ApplicationHeader()
{
}

uint32_t ApplicationHeader::getCrc(void) const
{
    uint32_t storedCrc = (static_cast<uint32_t>(_appSpace[CRC_OFFSET    ])      )
                       | (static_cast<uint32_t>(_appSpace[CRC_OFFSET + 1]) << 8 )
                       | (static_cast<uint32_t>(_appSpace[CRC_OFFSET + 2]) << 16)
                       | (static_cast<uint32_t>(_appSpace[CRC_OFFSET + 3]) << 24);

    return storedCrc;
}

uint32_t ApplicationHeader::getApplicationSize(void) const
{
    uint32_t appSize = (static_cast<uint32_t>(_appSpace[LENGTH_OFFSET    ])      )
                     | (static_cast<uint32_t>(_appSpace[LENGTH_OFFSET + 1]) << 8 )
                     | (static_cast<uint32_t>(_appSpace[LENGTH_OFFSET + 2]) << 16)
                     | (static_cast<uint32_t>(_appSpace[LENGTH_OFFSET + 3]) << 24);

    return appSize;
}

bool ApplicationHeader::isApplicationValid() const
{
    uint32_t appSize = getApplicationSize();
    if(appSize > (_appSpaceLength - APPLICATION_START_OFFSET))
    {
        return false;
    }

    uint32_t imageCRC = 0;

    Crc32_Normal crc;
    crc.update(&(_appSpace[HEADER_CRC_START_OFFSET]), HEADER_BYTES_TO_CRC);
    crc.update(&(_appSpace[APPLICATION_START_OFFSET]), appSize);
    imageCRC = crc.getCrc();

    return imageCRC == getCrc();
}

uint32_t ApplicationHeader::getApplicationStartAddress(uint32_t appSpaceAddress)
{
    return appSpaceAddress + APPLICATION_START_OFFSET;
}


} // namespace Firmware

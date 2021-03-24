#include <stddef.h>
#include <crc/Crc32.h>
#include <utilities_conf.h>
#include "firmware/ImageHeader.h"
#include <firmware/ApplicationHeader.h>

extern "C" {

void FWUpdate_execute(const uint8_t* appSpace, uint32_t appSpaceLength)
{
    Firmware::ImageHeader header(appSpace, appSpaceLength);
    if (header.isImageValid() == true)
    {
        DBGPRINTF("Header is valie\n");
    }
    else
    {
        DBGPRINTF("Header is garbage\n");
    }
}

}
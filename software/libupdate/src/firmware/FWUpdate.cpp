#include <stddef.h>
#include <crc/Crc32.h>
//#include <utilities_conf.h>
#include "firmware/ImageHeader.h"
#include <firmware/ApplicationHeader.h>
#include "debug_print.h"

extern "C" {

void FWUpdate_execute(const uint8_t* appSpace, uint32_t appSpaceLength)
{
    Firmware::ImageHeader header(appSpace, appSpaceLength);
    if (header.isImageValid() == true)
    {
        DEBUG_PRINTF(0, "Header is valid\n");
    }
    else
    {
        DEBUG_PRINTF(0, "Header is garbage\n");
    }
}

}
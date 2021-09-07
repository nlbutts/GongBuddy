#include <stddef.h>
#include <crc/Crc32.h>
//#include <utilities_conf.h>
#include "firmware/ImageHeader.h"
#include <firmware/ApplicationHeader.h>
#include "debug_print.h"

extern "C" {

/*
Normally the GongBuddy sends the heartbeat or impact message. The BaseStation
then responds with the ack/nak message. If a firmware update is requested
by the BaseStation, then in the ack/nak message, it will request a firmware
update. The GongBuddy will then transition into firmware update mode and
send a heartbeat message indicating it is ready for the firmware update image.

The BaseStation will then send ALL firmware update messages without waiting
for any ACKs. The GongBuddy will then send back a message with a bit field for
each packet. For example, if a firmware update is 50 KB, it will be split
into 50000 / 200 = 250 packets. The good/bad bit field uses 1 bit per packet.
A 1 is a good received packet and a 0 is a bad packet. So in this example,
there will be 250 bit fields packed 8 per byte or 32 bytes with bit fields.

The Base station will receive this bit field and resend only those packets.
This back and forth will repeat until all data has been received at the GongBuddy.
Then the GongBuddy will perform the firmware update and reboot.

The sequency diagram below shows how this works.
┌───────────────┐                ┌────────────────┐
│BaseStation    │                │ GongBuddy      │
│               │                │                │
└───────┬───────┘                └────────┬───────┘
        │  Status and Data messages       │
        │◄────────────────────────────────┤
        │                                 │
        │  Ack/Nak Feedback               │
        ├────────────────────────────────►│
        │                                 │
        │                                 │
        │  Ready for FW update            │
        │◄────────────────────────────────┤
        │ Packet 1                        │
        ├────────────────────────────────►│
        │ Packet 2                        │
        ├────────────────────────────────►│
        │ ...                             │
        │ Packet N                        │
        ├────────────────────────────────►│
        │                                 │
        │ Bit field of good/bad packets   │
        │◄────────────────────────────────┤
        │                                 │
        │ Retransmission of Packets X     │
        ├────────────────────────────────►│
        │                                 │
        │ Bit field of good/bad packets   │
        │◄────────────────────────────────┤
        │                                 │
        │                                 │
        │                                 │
        │                                 │
        │                                 │
        │                                 │
 */

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
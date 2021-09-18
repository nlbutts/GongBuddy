#include <stddef.h>
#include <crc/Crc32.h>
//#include <utilities_conf.h>
#include "firmware/ImageHeader.h"
#include <firmware/ApplicationHeader.h>
//#include "debug_print.h"
#include "firmware/FWUpdate.h"
#include "firmware/IFlash.h"

/*
Normally the GongBuddy sends the heartbeat or impact message. The BaseStation
then responds with the ack/nak message. If a firmware update is requested
by the BaseStation, then in the ack/nak message, it will request a firmware
update. The GongBuddy will then transition into firmware update mode and
send a heartbeat message indicating it is ready for the firmware update image.

The BaseStation will then send ALL firmware update messages without waiting
for any ACKs. The GongBuddy will then send back a message with a bit field for
each packet. For example, if a firmware update is 50 KB, it will be split
into 50000 / 240 (bytes/packet) = 209 packets.
The good/bad bit field uses 1 bit per packet. A 1 is a good received packet
and a 0 is a bad packet. So in this example, there will be 209 bit fields
packed 8 per byte or 27 bytes with bit fields (the last byte will not
use all of the bits).

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

FWUpdate::FWUpdate(IFlash &flash)
: _flash(&flash)
{

}

FWUpdate::~FWUpdate()
{

}

void FWUpdate::setFWProgInfo(uint32_t startAddress,
                             uint32_t numPackets,
                             uint32_t bytesPerPacket,
                             uint32_t crc)
{
    _startAddress = startAddress;
    _currentAddress = startAddress;
    _numPackets = numPackets;
    _bytesPerPacket = bytesPerPacket;
    _crc = crc;

    // Calculate how many pages will be required to store the data.
    _rxPacketBitField.resize((_numPackets / _bytesPerPacket) + 1);
}

bool FWUpdate::writePacket(uint32_t packetNum, uint8_t * data, uint32_t len)
{
    bool rv = false;
    if ((len % WriteChunkSize) == 0)
    {
        uint64_t * writeData = (uint64_t*)data;
        for (uint32_t i = 0; i < len; i += 2)
        {
            if (_flash->write(_currentAddress, *writeData))
            {
                writeData++;
                _currentAddress += 8; // 64 bit writes at a time
                recordPacket(packetNum);
                rv = true;
            }
        }
    }
    return rv;
}

int FWUpdate::getRxPacketBitField(uint8_t * buf, uint32_t len)
{

}

int FWUpdate::getBadPacketBitFIeld(uint8_t * buf, uint32_t len)
{

}


void FWUpdate::recordPacket(uint32_t packetNum)
{
    // Store the packet num in the bit field
    uint32_t offset = packetNum / 8;
    uint32_t mask = packetNum % 8;
    _rxPacketBitField[offset] |= (1 << mask);
}

bool FWUpdate::validateProgramming()
{
    return false;
}
/**
 * @file FWUpdate.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief Handles FW programming
 * @version 0.1
 * @date 2021-09-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef FW_UPDATE_H_
#define FW_UPDATE_H_

#include <stdint.h>
#include <vector>

class IFlash;

/**
 * @brief This class is responsible for handling the firmware update data
 * that is being received by the RF comm system
 *
 */
class FWUpdate
{
public:
    /**
     * @brief Construct a new FWUpdate object
     *
     */
    FWUpdate(IFlash &flash);
    /**
     * @brief Destroy the FWUpdate object
     *
     */
    ~FWUpdate();

    /**
     * @brief Initializes the program system, resets the received bit mask
     *
     * @param startAddress start address to program
     * @param numPackets number of expected packets
     * @param bytesPerPacket the number of bytes per packet
     * @param crc crc of the programmed code
     */
    void setFWProgInfo(uint32_t startAddress,
                       uint32_t numPackets,
                       uint32_t bytesPerPacket,
                       uint32_t crc);

    /**
     * @brief Write a packet into FLASH
     *
     * @param packetNum The packet number being received over the RF link
     * @param data a pointer to the data that was received
     * @param len the length of this chunk of data
     * @return true if the data was written to flash correctly
     * @return false if the data couldn't be written
     */
    bool writePacket(uint32_t packetNum, uint8_t * data, uint32_t len);

    /**
     * @brief Get the Rx Packet Bit Field data
     * @details returns a bit field in buf that indicates which packets
     * have been successfully received. The bits are packed starting at the
     * LSB in byte 0, continuing to the MSB, then back to the LSB of byte 1
     *
     * @param buf a pointer to the array that will receive the bit field
     * @param len the length of the buffer pointed to by buf
     * @return int the total number of bits packed into the bit field/buf array
     */
    int getRxPacketBitField(uint8_t * buf, uint32_t len);

    /**
     * @brief Get missing packets Bit Field data
     * @details returns a bit field in buf that indicates which packets
     * have been unsuccessfully received. The bits are packed starting at the
     * LSB in byte 0, continuing to the MSB, then back to the LSB of byte 1
     *
     * @param buf a pointer to the array that will receive the bit field
     * @param len the length of the buffer pointed to by buf
     * @return int the total number of bits packed into the bit field/buf array
     */
    int getBadPacketBitFIeld(uint8_t * buf, uint32_t len);

    /**
     * @brief Get the number of packets
     *
     * @return uint32_t packets
     */
    uint32_t getPackets()       {return _numPackets;}

    /**
     * @brief Validate that we have received all of the packets and the CRC
     * passes
     *
     * @return true everything is good, do the update
     * @return false everything is bad, panic, request packets
     */
    bool validateProgramming();

private:
    // Data has to be written in multiples of 2
    static constexpr uint32_t WriteChunkSize = 2;

private:
    void recordPacket(uint32_t packetNum);


private:
    IFlash                 *_flash;
    uint32_t                _startAddress;
    uint32_t                _currentAddress;
    uint32_t                _numPackets;
    uint32_t                _bytesPerPacket;
    uint32_t                _crc;
    std::vector<uint8_t>    _rxPacketBitField;
};

#endif /* FW_UPDATE_H_ */
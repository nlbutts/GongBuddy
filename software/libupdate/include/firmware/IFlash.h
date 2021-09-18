/**
 * @file IFlash.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief Interface class for FLASH interface
 * @version 0.1
 * @date 2021-09-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef IFLASH
#define IFLASH

/**
 * @brief This is the abstract base class for the FLASH system. It supports
 * erasing, reading, and writing FLASH memory
 *
 */
class IFlash
{
public:
    /**
     * @brief Construct a new IFlash object
     *
     */
    IFlash() {};
    /**
     * @brief Destroy the FWUpdate object
     *
     */
    virtual ~IFlash() {};

    /**
     * @brief Erase a sector in flash
     *
     * @param page the page number to erase
     * @return bool true if erased, false otherwise
     */
    virtual bool erasePage(uint32_t page) = 0;

    /**
     * @brief Writes a 32-bit value to FLASH
     *
     * @param address the address of the data to write
     * @param data the data to write
     * @return true if data was written successfully
     * @return false if data was not written
     */
    virtual bool write(uint32_t address, uint64_t data) = 0;

    /**
     * @brief Reads data from the address
     *
     * @param address address to read
     * @return uint32_t value at that address
     */
    virtual uint32_t read(uint32_t address) = 0;
};

#endif /* IFLASH */
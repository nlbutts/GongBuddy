/**
 * @file Crc32.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief CCITT32 CRC
 * @version 1.0
 * @date 2021-10-10
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef UTILITY_CRC_CRC32_H
#define UTILITY_CRC_CRC32_H

#include <stdint.h>

/**
 * @brief CRC interface class for update image validation and firmware validation
 */
class Crc32
{
public:
    /**
     * @brief Destructor
     */
    virtual ~Crc32();

    /**
     * @brief Resets the CRC to default
     */
    void reset(void);

    /**
     * @brief Updates the CRC with new data
     * @details This may be called multiple times as data becomes available,
     *          the CRC does not need to be calculated in a single operation
     *
     * @param data Pointer to the data to add in
     * @param len Length of the data to add in
     */
    void update(const uint8_t* data, uint32_t len);

    /**
     * @brief Gets the current CRC value
     *
     * @return CRC
     */
    uint32_t getCrc(void);

protected:
    typedef uint32_t table_t[256];

    /**
     * @brief Constructor
     * @note This will not initialize the default crc value. The derived class
     *       needs to call the reset() function from its constructor.
     */
    Crc32();

protected:
    /**
     * @brief Gets a reference to the CRC look up table
     *
     * @return Reference to the CRC look up table
     */
    virtual const table_t& _getCrcTable(void) = 0;

    /**
     * @brief Gets the initial CRC value
     *
     * @return Initial CRC value
     */
    virtual uint32_t _getInitValue(void) const = 0;

    /**
     * @brief Finalize the calculated CRC.
     *
     * @param crc CRC value to finalize
     * @return Finalized CRC value
     */
    virtual uint32_t _finalizeCrc(uint32_t crc) const = 0;

private:
    uint32_t _crc;
};


/**
 * @brief Normal CRC-32, polynomial 0xEDB88320
 * This is the same CRC32 as srecord -crc32-l-e
 */
class Crc32_Normal : public Crc32
{
public:
    Crc32_Normal()
    {
        // Reset must be called to properly initialize the default
        // CRC value. The base class is unable to do this because the initial
        // value depends on the derived class. (Constructors should never call
        // pure virtual functions)
        reset();
    }

private:
    const table_t& _getCrcTable(void)
    {
        return _table;
    }

    uint32_t _getInitValue(void) const
    {
        return 0xFFFFFFFF;
    }

    uint32_t _finalizeCrc(uint32_t crc) const
    {
        return crc ^ 0xFFFFFFFF;
    }

    static const Crc32::table_t _table;
};


#endif /* UTILITY_CRC_CRC32_H */
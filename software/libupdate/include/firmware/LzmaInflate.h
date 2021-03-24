#ifndef FIRMWARE_LZMA_INFLATE_H
#define FIRMWARE_LZMA_INFLATE_H

#include <stdint.h>
#include <lzma/LzmaDec.h>

namespace Firmware {

/**
 * @brief LZMA decompression class
 */
class LzmaInflate
{
public:
    enum State
    {
        OK,
        END_OF_STREAM,
        ALLOCATION_ERROR,
        STREAM_ERROR
    };

public:

    /**
     * @brief Constructor
     *
     * @param lzmaData Pointer to the compressed data
     * @param lzmaDataSize Size of the input LZMA data (compressed size in bytes)
     */
    LzmaInflate(const uint8_t* lzmaData, uint32_t lzmaDataSize);

    /**
     * @brief Destructor
     */
    virtual ~LzmaInflate();

    /**
     * @brief Read and decompress data into a buffer
     *
     * @param data Buffer to read into
     * @param len Number of bytes to read
     *
     * @return Number of bytes read
     */
    uint32_t read(uint8_t* data, uint32_t len);

    /**
     * @brief Gets the state of decompression (OK, Error, etc..)
     * @return Current decompression state
     */
    State getStreamState(void) const;

    /**
     * @brief Getter for LZMA library heap allocation statistics
     * @return Total number of bytes allocated
     */
    uint32_t getBytesAllocated(void) const;

    /**
     * @brief Getter for LZMA library heap allocation statistics
     * @return Total number of individual allocation requests
     */
    uint32_t getNumberOfAllocations(void) const;

private:
    // Internal error state
    State _internalState;

    // Pointer to the input data being decompressed
    const uint8_t* _input;

    // Size (in bytes) of data left in _data
    uint32_t _inputSize;

    // Size of the uncompressed data
    uint32_t _uncompressedSize;

    // Decompression properties
    uint8_t _props[LZMA_PROPS_SIZE];

    // Decompression state
    CLzmaDec _state;

    // Allocator to use in the lzma functions
    ISzAlloc _allocator;

private:
    static uint32_t _allocatedBytes;
    static uint32_t _allocationCount;

private:
    static void resetAllocationCounters(void);
    static void* lzmaAlloc(void *p, size_t size);
    static void lzmaFree(void *p, void *address);
};

} // namespace Firmware

#endif /* FIRMWARE_LZMA_INFLATE_H */
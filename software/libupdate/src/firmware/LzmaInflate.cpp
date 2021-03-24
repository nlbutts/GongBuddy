#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <firmware/LzmaInflate.h>

namespace Firmware {

uint32_t LzmaInflate::_allocatedBytes = 0;
uint32_t LzmaInflate::_allocationCount = 0;

LzmaInflate::LzmaInflate(const uint8_t* lzmaData, uint32_t lzmaDataSize) :
    _internalState(OK),
    _input(lzmaData),
    _inputSize(lzmaDataSize),
    _uncompressedSize(0)
{
    resetAllocationCounters();

    _uncompressedSize =    static_cast<uint32_t>( _input[0] )
                        | (static_cast<uint32_t>( _input[1]) << 8 )
                        | (static_cast<uint32_t>( _input[2]) << 16)
                        | (static_cast<uint32_t>( _input[3]) << 24);

    memcpy(_props, &_input[4], LZMA_PROPS_SIZE);

    _allocator.Alloc = lzmaAlloc;
    _allocator.Free = lzmaFree;

    LzmaDec_Construct(&_state);
    SRes res = LzmaDec_Allocate(&_state, _props, LZMA_PROPS_SIZE, &_allocator);
    if(res != SZ_OK)
    {
        _internalState = ALLOCATION_ERROR;
    }
    else
    {
        LzmaDec_Init(&_state);
    }

    _input += LZMA_PROPS_SIZE + 4;
    _inputSize -= LZMA_PROPS_SIZE + 4;
}

LzmaInflate::~LzmaInflate()
{
    _input = NULL;

    // If there was an allocator error, it is unknown if we can free the
    // memory successfully
    if(!(_internalState == ALLOCATION_ERROR))
    {
        LzmaDec_Free(&_state, &_allocator);
    }
}


uint32_t LzmaInflate::read(uint8_t* data, uint32_t len)
{
    if(_internalState != OK)
    {
        return 0;
    }

    SizeT outProcessed = len;
    SizeT inProcessed = _inputSize;

    ELzmaFinishMode finishMode = LZMA_FINISH_ANY;
    if(outProcessed >= _uncompressedSize)
    {
        finishMode = LZMA_FINISH_END;
        outProcessed = _uncompressedSize;
    }

    ELzmaStatus status = LZMA_STATUS_NOT_SPECIFIED;
    SRes res = LzmaDec_DecodeToBuf( &_state,        // Decoder state
                                    data,           // Destination buffer
                                    &outProcessed,  // Destination length
                                    _input,         // Source buffer
                                    &inProcessed,   // Source length
                                    finishMode,     // Finish mode
                                    &status);       // Status

    if(res != SZ_OK)
    {
        _internalState = STREAM_ERROR;
        return 0;
    }
    else
    {
        _input += inProcessed;
        _inputSize -= inProcessed;
        _uncompressedSize -= outProcessed;

        if(    (status == LZMA_STATUS_MAYBE_FINISHED_WITHOUT_MARK)
            || (status == LZMA_STATUS_FINISHED_WITH_MARK)
            || (_uncompressedSize == 0))
        {
            _internalState = END_OF_STREAM;
        }
    }

    return outProcessed;
}


LzmaInflate::State LzmaInflate::getStreamState() const
{
    return _internalState;
}

uint32_t LzmaInflate::getBytesAllocated(void) const
{
    return _allocatedBytes;
}

uint32_t LzmaInflate::getNumberOfAllocations(void) const
{
    return _allocationCount;
}

void LzmaInflate::resetAllocationCounters(void)
{
    _allocatedBytes = 0;
    _allocationCount = 0;
}

void* LzmaInflate::lzmaAlloc(void *p, size_t size)
{
    (void) p;

    _allocatedBytes += size;
    _allocationCount++;

    return malloc(size);
}

void LzmaInflate::lzmaFree(void *p, void *address)
{
    (void) p;

    free(address);
}


} // namespace Firmware

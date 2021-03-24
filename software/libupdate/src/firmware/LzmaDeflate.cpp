#include <string>
#include <iostream>
#include <stdint.h>
#include <stdlib.h>

#include <lzma/LzmaEnc.h>

#include <firmware/LzmaInflate.h>
using Firmware::LzmaInflate;

#include <firmware/LzmaDeflate.h>

namespace Firmware {

static SRes lzmaProgress(void *p, UInt64 inSize, UInt64 outSize);

LzmaDeflate::LzmaDeflate(const std::vector<unsigned char> &inputPayload) :
    _compressionRatio(0),
    _heapRequired(0),
    _success(false)
{
    _success = compressAndVerifyPayload(_compPayload, inputPayload);
    _compressionRatio = static_cast<float>(_compPayload.size()) / inputPayload.size();
}

LzmaDeflate::~LzmaDeflate()
{
}

bool LzmaDeflate::successful(void) const
{
    return _success;
}

const std::vector<unsigned char>& LzmaDeflate::getCompressedPaylod(void) const
{
    return _compPayload;
}

float LzmaDeflate::getCompressionRatio(void) const
{
    return _compressionRatio;
}

uint32_t LzmaDeflate::getDecompHeapRequired(void) const
{
    return _heapRequired;
}


bool LzmaDeflate::compressAndVerifyPayload(std::vector<unsigned char> &outBuf,
                                    const std::vector<unsigned char> &inBuf)
{
    // Compress the data
    if( !compressData(outBuf, inBuf) )
    {
        std::cerr << "Image compression failed\n";
        return false;
    }

    // Decompress the data to make sure it was done right
    std::vector<unsigned char> decompData;
    if( !decompressData(decompData, outBuf) )
    {
        std::cerr << "Image decompression failed\n";
        return false;
    }

    if( decompData.size() != inBuf.size() )
    {
        std::cerr << "Image compression/decompression failed\n";
        return false;
    }

    for(uint32_t i = 0; i < decompData.size(); ++i)
    {
        if(decompData[i] != inBuf[i])
        {
            std::cerr << "Decompressed data does not match image data\n";
            return false;
        }
    }

    return true;
}

bool LzmaDeflate::compressData(std::vector<unsigned char> &outBuf,
                        const std::vector<unsigned char> &inBuf)
{
    // Setup encoder properties
    CLzmaEncProps props;
    LzmaEncProps_Init( &props );
    props.dictSize      = DICTIONARY_SIZE;
    props.lp            = LZMA_LP;
    props.lc            = LZMA_LC;
    props.writeEndMark  = LZMA_WRITE_END_MARK ? 1 : 0;

    // Allocator
    ISzAlloc allocator = { LzmaDeflate::lzmaAlloc, LzmaDeflate::lzmaFree };

    // Progress function
    ICompressProgress progress = { lzmaProgress };

    // Decompression properties
    size_t propsSize = LZMA_PROPS_SIZE;
    size_t destLen = inBuf.size() + (inBuf.size() / 3) + 128;
    outBuf.resize(LENGTH_FIELD_SIZE + propsSize + destLen);

    // Save decompressed data size at the beginning of the data
    outBuf[0] = static_cast<unsigned char>((inBuf.size() >> 0 ) & 0xFF);
    outBuf[1] = static_cast<unsigned char>((inBuf.size() >> 8 ) & 0xFF);
    outBuf[2] = static_cast<unsigned char>((inBuf.size() >> 16) & 0xFF);
    outBuf[3] = static_cast<unsigned char>((inBuf.size() >> 24) & 0xFF);

    // Compress
    SRes res = LzmaEncode(  &outBuf[LENGTH_FIELD_SIZE + LZMA_PROPS_SIZE],
                            &destLen,
                            &inBuf[0],
                            inBuf.size(),
                            &props,
                            &outBuf[LENGTH_FIELD_SIZE],
                            &propsSize,
                            props.writeEndMark,
                            &progress,
                            &allocator,
                            &allocator
                            );

    if(res != SZ_OK)
    {
        return false;
    }

    outBuf.resize(LENGTH_FIELD_SIZE + propsSize + destLen);

    return true;
}


bool LzmaDeflate::decompressData(std::vector<unsigned char> &outBuf,
                          const std::vector<unsigned char> &inBuf)
{
    LzmaInflate decompressor(&inBuf[0], inBuf.size());

    LzmaInflate::State state = decompressor.getStreamState();
    if(state != LzmaInflate::OK)
    {
        std::cerr << "LZMA decompression constructor errror: "
                  << static_cast<int>(state) << std::endl;
        return false;
    }

    uint32_t bytesRead = 0;
    do
    {
        uint8_t out[DECOMPRESSION_CHUNK_SIZE];

        bytesRead = decompressor.read(out, DECOMPRESSION_CHUNK_SIZE);

        for(uint32_t i = 0; i < bytesRead; ++i)
        {
            outBuf.push_back(out[i]);
        }

        state = decompressor.getStreamState();
        switch(state)
        {
            case LzmaInflate::OK:
            case LzmaInflate::END_OF_STREAM:
                break;

            case LzmaInflate::ALLOCATION_ERROR:
                std::cerr << "Decompression allocation error\n";
                break;

            case LzmaInflate::STREAM_ERROR:
                std::cerr << "Decompression stream error. "
                          << "Decompressed " << outBuf.size() << " bytes before error\n";
                break;
        }

    } while((bytesRead != 0) && (state == LzmaInflate::OK));

    _heapRequired = decompressor.getBytesAllocated();
    return state == LzmaInflate::END_OF_STREAM;
}

void* LzmaDeflate::lzmaAlloc(void *p, size_t size)
{
    (void) p;
    return malloc(size);
}

void LzmaDeflate::lzmaFree(void *p, void *address)
{
    (void) p;
    free(address);
}

static SRes lzmaProgress(void *p, UInt64 inSize, UInt64 outSize)
{
    (void) p;
    (void) inSize;
    (void) outSize;

    return SZ_OK;
}

} // namespace Firmware

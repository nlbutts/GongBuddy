/**
 * Copyright (c) 2015 Appareo Systems, LLC.
 * 1810 NDSU Research Park Circle North
 * Fargo ND, 58102
 * All rights reserved.
 *
 * This is the confidential and proprietary information of Appareo Systems, LLC. You shall not
 * disclose such confidential information and shall use it only in accordance with the terms
 * of the license agreement you entered into with Appareo.
 *
 * File: LzmaDeflate.h
 * Creator: bschleusner
 * Date: May 12, 2015
 *
 * Copyright Version 1.0
 */

#ifndef APPAREO_IMAGEGEN_LZMA_DEFLATE_H
#define APPAREO_IMAGEGEN_LZMA_DEFLATE_H

#include <vector>

namespace Appareo {
namespace Imagegen {

/**
 * @brief Compressed payload creator
 */
class LzmaDeflate
{
public:
    /**
     * @brief Constructor
     * This constructor will compress the input payload, as well
     * as decompress it to verify that it works.
     *
     * @param inputPayload Payload to compress
     */
    LzmaDeflate(const std::vector<unsigned char> &inputPayload);
    virtual ~LzmaDeflate();

    /**
     * @brief Checks if the payload was successfully compressed and verified
     *
     * @return true on success, else failure
     */
    bool successful(void) const;

    /**
     * @brief Gets the compressed payload
     *
     * @return Compressed payload reference
     */
    const std::vector<unsigned char>& getCompressedPaylod(void) const;

    /**
     * @brief Gets the payload compression ratio
     *
     * @return (output size) / (input size)
     */
    float getCompressionRatio(void) const;

    /**
     * @brief Gets the number of heap bytes required to decompress
     *        the compressed payload
     *
     * @return Bytes of heap required to decompress payload
     */
    uint32_t getDecompHeapRequired(void) const;

private:
    std::vector<unsigned char>  _compPayload;
    float                       _compressionRatio;
    uint32_t                    _heapRequired;
    bool                        _success;

private:
    // The LP and LC variables will determine the maximum state buffer size
    // needed for decompression on the target.
    // state_size = (4 + (1.5 << (lc + lp))) KB
    static const int LZMA_LP = 0;
    static const int LZMA_LC = 0;

    // Compressed data end marker
    static const bool LZMA_WRITE_END_MARK = false;

    // Length field size (in bytes)
    static const size_t LENGTH_FIELD_SIZE = 4;

    // LZMA dictionary size
    static const uint32_t DICTIONARY_SIZE = (1 << 11); // 32k dictionary

    // Maximum block size to use when testing the image decompression
    static const uint32_t DECOMPRESSION_CHUNK_SIZE = 4096;

private:

    bool compressAndVerifyPayload(std::vector<unsigned char> &outBuf,
           const std::vector<unsigned char> &inBuf);

    bool decompressData(std::vector<unsigned char> &outBuf,
           const std::vector<unsigned char> &inBuf);

    bool compressData(std::vector<unsigned char> &outBuf,
           const std::vector<unsigned char> &inBuf);

    static void lzmaFree(void *p, void *address);
    static void* lzmaAlloc(void *p, size_t size);
};

} // namespace Imagegen
} // namespace Appareo

#endif /* APPAREO_IMAGEGEN_LZMA_DEFLATE_H */
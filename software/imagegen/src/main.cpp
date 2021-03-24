/**
 * Copyright (c) 2015 Appareo Systems, LLC.
 * 1810 NDSU Research Park Circle North
 * Fargo ND, 58102
 * All rights reserved.
 *
 * This is the confidential and proprietary information of Appareo Systems,
 * LLC. You shall not disclose such confidential information and shall use it
 * only in accordance with the terms of the license agreement you entered into
 * with Appareo.
 *
 * File: main.cpp
 * Creator: bschleusner
 * Date: May 7, 2015
 *
 * Copyright Version 1.0
 */
#include <stdint.h>

#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include <tclap/CmdLine.h>

extern "C" {
#include <BuildConstants.h>

}

#include <firmware/ImageHeader.h>
using Firmware::ImageHeader;

#include <firmware/LzmaDeflate.h>
using Firmware::LzmaDeflate;


// --------------------------------------------------- Local functions and data

namespace {

    bool readFile(  const std::string filename,
                    std::vector<unsigned char> &outBuf)
    {
        std::ifstream file(filename.c_str(), std::ios::binary);
        file.unsetf(std::ios::skipws);

        if(!file.good())
        {
            return false;
        }

        std::streampos fileSize;
        file.seekg(0, std::ios::end);
        fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        outBuf.clear();
        outBuf.reserve(fileSize);

        outBuf.insert(outBuf.begin(),
                   std::istream_iterator<unsigned char>(file),
                   std::istream_iterator<unsigned char>());

        return true;
    }


    bool writeFile( const std::string filename,
                    const std::vector<unsigned char> &inBuf)
    {
        std::ofstream file(filename.c_str(), std::ios::binary);
        file.unsetf(std::ios::skipws);

        if(!file.good())
        {
            return false;
        }

        for(uint32_t i = 0; i < inBuf.size(); ++i)
        {
            file << inBuf[i];
        }

        return true;
    }


    ImageHeader::Type getImageType(bool compressImage, bool isApplication)
    {
        ImageHeader::Type imageType = ImageHeader::INVALID;
        if(isApplication)
        {
            if(compressImage)
            {
                imageType = ImageHeader::APPLICATION_LZMA;
            }
            else
            {
                imageType = ImageHeader::APPLICATION;
            }
        }
        else // is bootloader
        {
            if(compressImage)
            {
                imageType = ImageHeader::BOOTLOADER_LZMA;
            }
            else
            {
                imageType = ImageHeader::BOOTLOADER;
            }
        }

        return imageType;
    }

} // namespace


// ----------------------------------------------------------------------- Main

int main(int argc, char **argv)
{
    std::string inputFilename;
    std::string outputFilename;

    bool compressImage = true;
    bool isApplication = true;
    uint32_t hwCompatField = 0;

    // -------------------------------- Argument parsing
    try
    {
        // Add in arguments
        TCLAP::CmdLine cmd("Abercrombie Firmware Image Generator -\
            See https://documentation.appareo.com/display/FRON/\
            MADS+Firmware+Update+Architecture for details",
            ' ', "UNKNOWN");

        TCLAP::UnlabeledValueArg<std::string>
            inputFileParam( "input",
                            "Input filename, binary format",
                            true,
                            "",
                            "input");
        cmd.add( inputFileParam );

        TCLAP::UnlabeledValueArg<std::string>
            outputFileParam("output",
                            "Output filename, binary update image",
                            true,
                            "",
                            "output" );
        cmd.add( outputFileParam );

        TCLAP::SwitchArg isAppParam("a", "application", "Create application image");
        TCLAP::SwitchArg isBootParam("b", "bootloader", "Create bootloader image");
        cmd.xorAdd( isAppParam, isBootParam );

        TCLAP::SwitchArg compressParam("c", "compress", "Compress image with LZMA");
        cmd.add( compressParam );

        TCLAP::ValueArg<uint32_t> hwCompatParam("w", "hardware",
            "Hardware compatibility bitfield. See confluence for detailed usage.",
            false,
            getHardwareCompatBitfield(),
            "bitfield");
        cmd.add( hwCompatParam );

        // Parse the argv array.
        cmd.parse( argc, argv );

        // Save the results
        inputFilename = inputFileParam.getValue();
        outputFilename = outputFileParam.getValue();
        hwCompatField = hwCompatParam.getValue();
        compressImage = compressParam.getValue();
        if(isAppParam.getValue())
        {
            isApplication = true;
        }
        else if(isBootParam.getValue())
        {
            isApplication = false;
        }
        else
        {
            std::cerr << "! Argument parsing failure\n";
            return -1;
        }

        if(0 == hwCompatField)
        {
            std::cerr << "! Hardware compatibility field cannot be 0\n";
            return -1;
        }
    }
    catch (TCLAP::ArgException &e)
    {
        std::cerr << "! Error: " << e.error() << " for arg " << e.argId() << std::endl;
        return -1;
    }


    // -------------------------------- Print operation details
    {
        std::ios::fmtflags flags(std::cout.flags());
        std::cout
            << "= Compression: " << (compressImage ? "Enabled" : "Disabled")
            << std::endl
            << "= Firmware Type: " << (isApplication ? "Application" : "Bootloader")
            << std::endl
            << "= Hardware compatibility bitfield: 0x"
            << std::hex << std::setw(8) << std::setfill('0') << hwCompatField
            << std::endl;
        std::cout.flags(flags);
    }


    // -------------------------------- Load the input file into a vector
    std::cout << "> Reading " << inputFilename << std::endl;
    std::vector<unsigned char> rawData;
    if( !readFile(inputFilename, rawData) )
    {
        std::cerr << "! Could not open file '" << inputFilename << "' for reading.\n";
        return -1;
    }

    std::cout << "= Input file size (bytes): " << rawData.size() << std::endl;


    // -------------------------------- Create payload
    std::vector<unsigned char> payload;
    payload.clear();

    if(compressImage)
    {
        LzmaDeflate compressor(rawData);

        if( !compressor.successful() )
        {
            std::cerr << "! Compressed failed.\n";
            return -1;
        }

        payload = compressor.getCompressedPaylod();

        std::cout
            << "= Compression ratio: "
            << compressor.getCompressionRatio() << std::endl
            << "= Heap required for decompression (bytes): "
            << compressor.getDecompHeapRequired()
            << std::endl;
    }
    else
    {
        // No compression
        payload = rawData;
    }


    // -------------------------------- Get the image type ID
    ImageHeader::Type imageType = getImageType(compressImage, isApplication);

    std::vector<uint8_t> serialImage(ImageHeader::IMAGE_HEADER_LENGTH);
    serialImage.clear();


    // -------------------------------- Create image header
    {
        ImageHeader header(imageType, hwCompatField,
                           &(payload[0]), payload.size());

        uint8_t serialized[ImageHeader::IMAGE_HEADER_LENGTH];
        header.serializeHeader(serialized);

        serialImage.insert( serialImage.end(),
                            &serialized[0],
                            &serialized[ImageHeader::IMAGE_HEADER_LENGTH]);
    }


    // -------------------------------- Append the payload to the header
    serialImage.insert(serialImage.end(), payload.begin(), payload.end());


    // -------------------------------- Save the update image to disk
    std::cout << "< Writing " << outputFilename << std::endl;
    if( !writeFile(outputFilename, serialImage) )
    {
        std::cerr << "! Could not open file '" << outputFilename << "' for writing.\n";
        return -1;
    }

    std::cout << "= Output file size (bytes): " << serialImage.size() << std::endl;

    return 0;
}

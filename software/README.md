# GongBuddy
A Cmake build project for ST SensorTile

# Overview
This project is intended to create a CMake environment for the ST SensorTile dev kit. ST has example projects and files for various IDEs.
I'm a bit of a purist and would rather use CMake and whatever editor I want.

This could use more work, but right now I build two libraries: driver, middleware, and one application. The include paths are a bit
much and need to be paired down, but ST likes to have a lot of include dependencies.
There is a dependency on your embedded toolchain. The CMake system expects you to provide a toolchain file. An example is included
in the toolchain directory.

# Dependencies
[Srecord 1.64] (http://srecord.sourceforge.net/)

[CMake 3.7 or newer] (https://cmake.org/download/)



# Building
Set the path to the toolchain in the environment

```
export TOOLCHAIN_PATH=~/gcc-arm-none-eabi-9-2020-q2-update/
```

```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain/arm_embedded_toolchain.txt ..
make -j99
```

# LoRa Communications

The system uses a LoRa radio, but does not use LoraWan protocol. Instead the
system uses a protobufs system. The entire data field of the LoRa packet
is a protobuf. The source and destination address are present in there.

There is already a CRC on the LoRa packet, therefore additional data integrity
checks are not present.

# Firmware Update
The raw binary firmware image is compressed using LZMA and then wrapped in an
image header. The image header is defined below:

| Element | Size | Description |
| -- | -- | -- |
| SUM16 | 2 | This is a simple 16-bit sum of image header |
| CCITT-32 CRC | 4 | This is a CCITT-32 CRC of the next element and payload |
| Payload length | 4 | length of the entire payload |
| Image type | 1 | 0=Invalid, 1 = App, 2 = App LZMA |
| HW compatibility | 4 | A 32-bit field that is used to indicate compatiblity a hardware platform |
| Reserved | 17 | |
| Payload | N | The uncompressed or compressed payload |
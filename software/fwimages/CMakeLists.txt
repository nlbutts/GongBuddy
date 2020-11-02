# ------------------------------------------------------- MINIMUM CMAKE VERSION
cmake_minimum_required(VERSION 3.4)


# ---------------------------------------------------------- ONLY BUILD FOR ARM
if(NOT ${CMAKE_SYSTEM_PROCESSOR} MATCHES arm_baremetal)
    message(FATAL_ERROR "${CMAKE_CURRENT_LIST_FILE} can only be built for arm_baremetal!")
endif()


# ----------------------------------------------------- BUILD APPLICATION FILES
# Built in application/CMakeLists.txt
set(APPLICATION_ELF application.elf)
set(APPLICATION_DEBUG_ELF application_debug.elf)

# Built locally
set(APPLICATION_BIN application.bin)
set(APPLICATION_DEBUG_HEX application_debug.hex)
set(APPLICATION_CRC_HEX application_crc.hex)
set(APPLICATION_CRC_BIN application_crc.bin)

add_custom_command(
    OUTPUT ${APPLICATION_BIN}
    COMMAND ${OC} -O binary $<TARGET_FILE:${APPLICATION_ELF}> ${APPLICATION_BIN}
    DEPENDS ${APPLICATION_ELF}
    COMMENT "Building ${APPLICATION_BIN}"
)

add_custom_command(
    OUTPUT ${APPLICATION_DEBUG_HEX}
    COMMAND ${OC} -O ihex $<TARGET_FILE:${APPLICATION_DEBUG_ELF}> ${APPLICATION_DEBUG_HEX}
    DEPENDS ${APPLICATION_DEBUG_ELF}
    COMMENT "Building ${APPLICATION_DEBUG_HEX}"
)

add_custom_command(
    OUTPUT ${APPLICATION_CRC_HEX}
    COMMAND ${SREC_CAT} ${APPLICATION_HEX} -intel
                -exclusive-length-l-e 0x08040004
                -fill 0xFF 0x08040008 0x08040200
                -crc32_little_endian 0x08040000
                -o ${APPLICATION_CRC_HEX} -intel
    DEPENDS ${APPLICATION_HEX}
    COMMENT "Building ${APPLICATION_CRC_HEX}"
)

add_custom_command(
    OUTPUT ${APPLICATION_CRC_BIN}
    COMMAND ${SREC_CAT} ${APPLICATION_CRC_HEX} -intel
                -offset -0x08040000
                -o ${APPLICATION_CRC_BIN} -binary
    DEPENDS ${APPLICATION_CRC_HEX}
    COMMENT "Building ${APPLICATION_CRC_BIN}"
)


# ------------------------------------------------------- BUILD BOOTBLOCK FILES
# Built in bootblock/CMakeLists.txt
# set(BOOTBLOCK_ELF bootblock.elf)

# # Built locally
# set(BOOTBLOCK_HEX bootblock.hex)
# set(BOOTBLOCK_BIN bootblock.bin)

# add_custom_command(
#     OUTPUT ${BOOTBLOCK_HEX}
#     COMMAND ${OC} -O ihex $<TARGET_FILE:${BOOTBLOCK_ELF}> ${BOOTBLOCK_HEX}
#     DEPENDS ${BOOTBLOCK_ELF}
#     COMMENT "Building ${BOOTBLOCK_HEX}"
# )

# add_custom_command(
#     OUTPUT ${BOOTBLOCK_BIN}
#     COMMAND ${SREC_CAT} ${BOOTBLOCK_HEX} -intel
#                 -offset -0x08000000
#                 -o ${BOOTBLOCK_BIN} -binary
#     DEPENDS ${BOOTBLOCK_HEX}
#     COMMENT "Building ${BOOTBLOCK_BIN}"
# )


# # -------------------------------------------------------- BUILD ASSEMBLY IMAGE
# # Built locally
# set(ASSEMBLY_HEX assembly.hex)

# # This pulls in bootblock and application into a single hex file. Note that
# # application is pulled in twice, once for the backup image and once for the
# # run location
# add_custom_command(
#     OUTPUT ${ASSEMBLY_HEX}
#     COMMAND ${SREC_CAT} ${BOOTBLOCK_HEX} -intel
#                 ${APPLICATION_CRC_HEX} -intel -offset -0x000020000
#                 ${APPLICATION_CRC_HEX} -intel
#                 -o ${ASSEMBLY_HEX} -intel
#     DEPENDS ${BOOTBLOCK_HEX} ${APPLICATION_CRC_HEX}
#     COMMENT "Building ${ASSEMBLY_HEX}"
# )


# # ------------------------------------------------- SET TARGETS TO BUILD ON ALL
add_custom_target(fwimages ALL
    DEPENDS ${APPLICATION_BIN}
)

# ------------------------------------------------------------- INSTALL TARGETS
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${APPLICATION_BIN}
        RENAME application-${ASSEMBLY_PN}-${BUILD_VERSION}.bin
        DESTINATION ./
)

# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${ASSEMBLY_HEX}
#         RENAME assembly-${ASSEMBLY_PN}-${BUILD_VERSION}.hex
#         DESTINATION ./
# )

# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${APPLICATION_CRC_HEX}
#         RENAME application_crc-${APP_PN}-${BUILD_VERSION}.hex
#         DESTINATION ./
# )

# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${BOOTBLOCK_HEX}
#         RENAME bootblock-${BOOT_PN}-${BUILD_VERSION}.hex
#         DESTINATION ./
# )

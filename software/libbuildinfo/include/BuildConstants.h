#ifndef BUILD_CONSTANTS_H
#define BUILD_CONSTANTS_H


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * These constants are passed in at compile time. See the CMakeLists.txt file
 * to see how that is done.
 *
 * Note that the part numbers and version strings should only be used to
 * determine the version of the current executable, for example, bootblock may
 * use the information in here to determine its own version, but it should NOT
 * use the information in here to determine applications version, as bootblock
 * has no idea if the application that is running was built at the same time
 * as itself.
 *
 * Information that needs to be crossed between application and
 * bootblock should be done through the MessagePassing::Manager instead.
 */


/**
 * @brief Gets the firmware version string, i.e. "1.0.0.1051"
 * @return Pointer to the version string
 */
const char * getBuildVersionString(void);

/**
 * @brief Gets the version control system commit hash of this build, i.e.
 *          "53dc4e5fb357299fdf1955c1b57ddff0fef96161+"
 * @return Pointer to the version control system commit hash string
 */
const char * getCommitHashString(void);

/**
 * @brief Gets the firmware build date string
 * @return Pointer to the build date string
 */
const char * getBuildDateString(void);

/**
 * @brief Gets the hardware compatibility bit-field for this firmware
 * @details See the System ICD for a detailed explanation of this field. This
 *          constant is used by NV data to provide a sane constant, as well as
 *          during the update image generation.
 * @return Hardware compatibility field for this build
 */
uint32_t getHardwareCompatBitfield(void);


#ifdef __cplusplus
}
#endif

#endif /* BUILD_CONSTANTS_H */

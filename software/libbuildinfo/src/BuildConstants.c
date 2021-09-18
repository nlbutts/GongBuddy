#include <BuildConstants.h>

#include <stddef.h>


#ifdef BUILD_VERSION
    #define FW_VERSION_STR BUILD_VERSION
#else
    #define FW_VERSION_STR "1"
    #warning BUILD_VERSION is undefined
#endif

#ifdef VCS_HASH
    #define FW_VCS_HASH_STR VCS_HASH
#else
    #define FW_VCS_HASH_STR ""
    #warning VCS_HASH is undefined
#endif

#ifdef BUILD_DATE
    #define FW_DATE_STR BUILD_DATE
#else
    #define FW_DATE_STR ""
    #warning BUILD_DATE is undefined
#endif

#ifdef HW_COMPAT_BITFIELD
    #define FW_COMPAT_BITFIELD HW_COMPAT_BITFIELD
#else
    #define FW_COMPAT_BITFIELD 0
    #warning HW_COMPAT_BITFIELD is undefined
#endif


const char * getBuildVersionString(void)
{
    return FW_VERSION_STR;
}

const char * getCommitHashString(void)
{
    return FW_VCS_HASH_STR;
}

const char * getBuildDateString(void)
{
    return FW_DATE_STR;
}

uint32_t getHardwareCompatBitfield(void)
{
    return FW_COMPAT_BITFIELD;
}

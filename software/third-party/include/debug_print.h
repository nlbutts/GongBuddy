/**
 * @file debug_print.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief Remapper for Segger RTT
 * @version 1.0
 * @date 2021-07-31
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef DEBUG_PRINT_H_
#define DEBUG_PRINT_H_

#if defined(ENABLE_DEBUG_PRINT)
#include "SEGGER_RTT.h"
#define DEBUG_PRINTF SEGGER_RTT_printf
#else
#define DEBUG_PRINTF(BufferIndex, sFormat, ...)
//int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
#endif

#endif /* DEBUG_PRINT_H_ */

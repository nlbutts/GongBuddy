/**
 * @file IFlash_mock.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief Mock for the FLASH interface
 * @version 0.1
 * @date 2021-09-12
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <gmock/gmock.h>

#include "firmware/IFlash.h"

class IFlash_mock : public IFlash {
 public:
  MOCK_METHOD(bool, erasePage, (uint32_t page), (override));
  MOCK_METHOD(bool, write, (uint32_t address, uint64_t data), (override));
  MOCK_METHOD(uint32_t, read, (uint32_t address), (override));
};
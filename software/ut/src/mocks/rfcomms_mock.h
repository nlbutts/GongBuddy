/**
 * @file rfcomms.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief This is the class that is used to exchange data over the LoRa radio
 * @version 1.0
 * @date 2021-09-06
 *
 * @copyright Copyright (c) 2021
 *
 *
 * TODO:
 */

#include <vector>
#include <stdint.h>
#include <gmock/gmock.h>
#include "Irfcomms.h"

class MockRFComms : public IRFComms {
 public:
  MOCK_METHOD(int, sendData, (uint8_t * pbData, int length), (override));
  MOCK_METHOD(int, getData, (uint8_t * pbData, int length, int timeout), (override));
};
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
#include "Irfcomms.h"

class RFComms : public IRFComms
{
public:
    /**
     * @brief Constructor
     *
     */
    RFComms();
    /**
     * @brief Destroy the RFComms object
     *
     */
    ~RFComms();

    int sendData(uint8_t * pbData, int length) override;

    int getData(uint8_t * pbData, int length, int timeout) override;
};
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
    /**
     * @brief Send data to the host
     *
     * @param pbData serilized protobuf
     * @return int number of bytes Txed
     */
    int sendData(std::vector<uint8_t> pbData) override;
    /**
     * @brief Get data from the host
     *
     * @param pbData serilized protobuf
     * @param timeout timeout in milliseconds
     * @return int number of bytes Received
     */
    int getData(std::vector<uint8_t> &pbData, int timeout) override;
};
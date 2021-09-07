/**
 * @file rfcomms.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief Abstract base class for some sort of communications link
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

class IRFComms
{
public:
    /**
     * @brief Destroy the RFComms object
     *
     */
    ~IRFComms();
    /**
     * @brief Send data to the host
     *
     * @param pbData serilized protobuf
     * @return int number of bytes Txed
     */
    virtual int sendData(std::vector<uint8_t> pbData) = 0;
    /**
     * @brief Get data from the host
     *
     * @param pbData serilized protobuf
     * @param timeout timeout in milliseconds
     * @return int number of bytes Received
     */
    virtual int getData(std::vector<uint8_t> &pbData, int timeout) = 0;

protected:
    /**
     * @brief Constructor, make protected, so this class can't be created
     *
     */
    IRFComms() {};
};
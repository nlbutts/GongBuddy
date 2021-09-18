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

class IRFComms
{
public:
    /**
     * @brief Destroy the RFComms object
     *
     */
    ~IRFComms(){};
    /**
     * @brief Send data to the host
     *
     * @param pbData pointer to a buffer of data
     * @param length the length of data to transmit
     * @return int number of bytes Txed
     */
    virtual int sendData(uint8_t * pbData, int length) = 0;
    /**
     * @brief Get data from the host
     *
     * @param pbData pointer to a buffer to receive the data
     * @param length the length of the buffer
     * @param timeout timeout in milliseconds
     * @return int number of bytes Received
     */
    virtual int getData(uint8_t * pbData, int length, int timeout) = 0;

protected:
    /**
     * @brief Constructor, make protected, so this class can't be created
     *
     */
    IRFComms() {};
};
/**
 * @file comm_mngr.h
 * @author Nick Butts (nlbutts@ieee.org)
 * @brief This class will transmit and receive data through the rfcomms system.
 * @details When data needs to be sent, the data will come here and be placed
 * into a protobuf, then serialized, than transmitted. It will then wait
 * the timeout until data comes back or a timeout occurs. When data comes
 * back, it will determine where to route the messages
 * @version 0.1
 * @date 2021-09-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef COMM_MNGR_H_
#define COMM_MNGR_H_

#include <vector>
#include <stdint.h>
#include "gb_messages.pb.h"

class IRFComms;
class ITimer;
class FWUpdate;

class CommManager
{
public:
    /**
     * @brief Constructor
     *
     * @param uniqueID a 32-bit unique ID
     * @param comms a reference to the underlying communication device
     * @param timer a reference to a generic timer
     * @param timeout the timeout between sending data to the comms and receiving data
     * @param fwupdate a reference to the firmware update object
     *
     */
    CommManager(uint32_t uniqueID,
                IRFComms & comms,
                ITimer &timer,
                int timeout,
                FWUpdate &fwupdate);

    /**
     * @brief Destroy the CommManager object
     *
     */
    ~CommManager();

    /**
     * @brief Send data back to the base station without IMU, I.E. Heartbeat
     * @details This function will take the various data parameters
     * place them into a protobuf message, serialize the message
     * and transmit the data via Lora
     *
     * @param pressure - pressure in millibars
     * @param temperature - temperature in deg C
     * @param batteryVoltage - battery voltage in millivolts
     * @param threshold - The impact threshold in milligees
     * @param configuration - The bit field configuration
     * @param rssi - The RSSI from the received signal
     * @return int - 0 if successful, otherwise error code
     */
    int sendData(uint32_t pressure,
                 uint32_t temperature,
                 uint32_t batteryVoltage,
                 uint32_t threshold,
                 uint32_t configuration,
                 int32_t rssi);

    /**
     * @brief Send data back to the base station with impact data
     * @details This function will take the various data parameters
     * place them into a protobuf message, serialize the message
     * and transmit the data via Lora
     *
     * @param pressure - pressure in millibars
     * @param temperature - temperature in deg C
     * @param batteryVoltage - battery voltage in millivolts
     * @param threshold - The impact threshold in milligees
     * @param configuration - The bit field configuration
     * @param rssi - The RSSI from the received signal
     * @param imu - vector of IMU samples
     * @return int - 0 if successful, otherwise error code
     */
    int sendData(uint32_t pressure,
                 uint32_t temperature,
                 uint32_t batteryVoltage,
                 uint32_t threshold,
                 uint32_t configuration,
                 int32_t rssi,
                 std::vector<int16_t> &imu);

    /**
     * @brief This function attempts to receive data from the host device
     * @details After SendData is called, this function should be called.
     * It waits the timeout value for new data from the host. If not data is
     * received it returns. If data is received it processes the data
     *
     * @return bool true the callee should continue to call the function,
     * received.
     */
    bool processResponse();

    /**
     * @brief Get the Last Status object
     *
     * @return Status
     */
    Status getLastStatus()  {return _lastStatus;}

private:
    /**
     * @brief Sends the FW update status message
     *
     * @param status the status to send
     * @return bool follows the _comms->send return
     */
    bool sendFWUpdateStatus(FWUpdateStatus_FWStatus status);

    void populateBaseData(LoraMsg2 &msg,
                          uint32_t pressure,
                          uint32_t temperature,
                          uint32_t batteryVoltage,
                          uint32_t threshold,
                          uint32_t configuration,
                          int32_t rssi);

    void populateIMUData(LoraMsg2 &msg,
                         std::vector<int16_t> &imu);

    int transmitPB(LoraMsg2 &msg);

private:
    uint32_t    _uniqueID;  // Pointer to unique ID
    IRFComms   *_comms;     // Pointer to RF comms
    ITimer     *_timer;
    int         _timeout;   // timeout in milliseconds
    FWUpdate   *_fwupdate;
    Status      _lastStatus;
};

#endif // COMM_MNGR_H_
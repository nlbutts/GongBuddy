#include <stdlib.h>
#include "comm_mngr.h"
#include "BuildConstants.h"
#include <pb_encode.h>
#include <pb_decode.h>
#include "firmware/FWUpdate.h"
#include "Irfcomms.h"
#include "ITimer.h"

CommManager::CommManager(uint32_t uniqueID,
                         IRFComms & comms,
                         ITimer &timer,
                         int timeout,
                         FWUpdate &fwupdate)
: _uniqueID(uniqueID)
, _comms(&comms)
, _timer(&timer)
, _timeout(timeout)
, _fwupdate(&fwupdate)
{

}

CommManager::~CommManager()
{

}

int CommManager::sendData(uint32_t pressure,
                          uint32_t temperature,
                          uint32_t batteryVoltage,
                          uint32_t threshold,
                          uint32_t configuration,
                          int32_t rssi)
{
    LoraMsg2 msg = LoraMsg2_init_default;
    populateBaseData(msg,
                     pressure,
                     temperature,
                     batteryVoltage,
                     threshold,
                     configuration,
                     rssi);
    return transmitPB(msg);
}


int CommManager::sendData(uint32_t pressure,
                          uint32_t temperature,
                          uint32_t batteryVoltage,
                          uint32_t threshold,
                          uint32_t configuration,
                          int32_t rssi,
                          std::vector<int16_t> &imu)
{
    LoraMsg2 msg = LoraMsg2_init_default;
    populateBaseData(msg,
                     pressure,
                     temperature,
                     batteryVoltage,
                     threshold,
                     configuration,
                     rssi);
    populateIMUData(msg, imu);
    return transmitPB(msg);
}

void CommManager::populateBaseData(LoraMsg2 &msg,
                                   uint32_t pressure,
                                   uint32_t temperature,
                                   uint32_t batteryVoltage,
                                   uint32_t threshold,
                                   uint32_t configuration,
                                   int32_t rssi)
{
    auto build = atoi(getBuildVersionString());
    msg.status = Status_HEARTBEAT;
    msg.gonginfo.dev_id = _uniqueID;
    msg.gonginfo.buildnum = build;
    msg.gonginfo.pressure = pressure;
    msg.gonginfo.temperature = temperature;
    msg.gonginfo.batt_voltage = batteryVoltage;
    msg.gonginfo.threshold = threshold;
    msg.gonginfo.configuration = configuration;
    msg.gonginfo.rssi = rssi;
    msg.has_gonginfo = true;
}

void CommManager::populateIMUData(LoraMsg2 &msg,
                                  std::vector<int16_t> &imu)
{
    // I know this is dangerous, but I just want to get a pointer to the raw data
    uint8_t * src = (uint8_t*)imu.data();
    // Now copy that data into the struct
    memcpy(msg.gonginfo.imu.bytes, src, imu.size()*2);
    msg.gonginfo.imu.size = imu.size() * 2;
    msg.status = Status_IMPACT;
}

int CommManager::transmitPB(LoraMsg2 &msg)
{
    uint8_t buffer[250];

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    pb_encode(&stream, LoraMsg2_fields, &msg);
    return _comms->sendData(buffer, stream.bytes_written);
}

bool CommManager::processResponse()
{
    uint8_t buffer[250];
    int rxBytes = _comms->getData(buffer, 250, _timeout);
    if (rxBytes > 0)
    {
        // Decode the data
        LoraMsg2 msg = LoraMsg2_init_default;
        pb_istream_t stream = pb_istream_from_buffer(buffer, rxBytes);
        if (pb_decode(&stream, LoraMsg2_fields, &msg))
        {
            _lastStatus = msg.status;

            if (msg.has_fw_setup)
            {
                _timer->setTimerMs(CommManager::ProgrammingTimeout);
                _fwupdate->setFWProgInfo(msg.fw_setup.start_address,
                                         msg.fw_setup.total_packets,
                                         msg.fw_setup.bytes_per_packet,
                                         msg.fw_setup.fw_crc);

                // Send a message indicating we are ready for data
                sendFWUpdateStatus(Status_REPROGRAMMING, FWUpdateStatus_FWStatus_READY_FOR_PAYLOAD);
                return true;
            }
            else if (msg.has_reprog)
            {
                _timer->setTimerMs(CommManager::ProgrammingTimeout);
                _fwupdate->writePacket(msg.reprog.packet, msg.reprog.data.bytes, msg.reprog.data.size);
                if (msg.reprog.packet == (_fwupdate->getPackets() - 1))
                {
                    if (_fwupdate->validateProgramming())
                    {
                        sendFWUpdateStatus(Status_REPROGRAMMING, FWUpdateStatus_FWStatus_VALID_FW_BLOB);
                    }
                    else
                    {
                        sendFWUpdateStatus(Status_REPROGRAMMING, FWUpdateStatus_FWStatus_INVALID_CRC);
                    }
                }
                return true;
            }
        }
    }
    else if (_timer->isTimerExpired() && (_lastStatus == Status_REPROGRAMMING))
    {
        // TODO: Send packets that are missing
        sendFWUpdateStatus(Status_REPROGRAMMING, FWUpdateStatus_FWStatus_MISSING_PACKETS);
    }
    return false;
}

bool CommManager::sendFWUpdateStatus(Status status, FWUpdateStatus_FWStatus fwstatus)
{
    LoraMsg2 msg = LoraMsg2_init_default;
    uint8_t buf[10];
    msg.status = status;
    msg.has_fw_status = true;
    msg.fw_status.status = fwstatus;
    pb_ostream_t ostream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (pb_encode(&ostream, LoraMsg2_fields, &msg))
    {
        if (_comms->sendData(buf, ostream.bytes_written) > 0)
        {
            return true;
        }
    }
    return false;
}


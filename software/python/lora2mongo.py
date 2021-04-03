import digitalio
import board
import busio
import adafruit_rfm9x
import time
import gb_messages_pb2
import time
import pymongo
import datetime
import struct
import os
import re

def formatImu(pb):
    imuData = {}
    if pb.HasField('imu'):
        data = pb.imu
        index = 0
        accelx = []
        accely = []
        accelz = []
        gyrox = []
        gyroy = []
        gyroz = []

        while index < len(data):
            sample = struct.unpack('hhhhhh', data[index:index+12])
            accelx.append(sample[0])
            accely.append(sample[1])
            accelz.append(sample[2])
            gyrox.append( sample[3])
            gyroy.append( sample[4])
            gyroz.append( sample[5])
            index += 12

        imuData['accelx'] = accelx
        imuData['accely'] = accely
        imuData['accelz'] = accelz
        imuData['gyrox'] = gyrox
        imuData['gyroy'] = gyroy
        imuData['gyroz'] = gyroz
    print(imuData)
    return imuData

def processGongData(db, pb, rssi):
    imuData = formatImu(pb)
    q = {"device_id": pb.identifier}
    res = db.find(q)
    if res.count() == 0:
        print('No device in DB, creating new one')
        mydict = {
                'name': 'Name me please',
                'device_id': pb.identifier,
                'build_num': pb.buildnum,
                'temperature': pb.temperature,
                'batt_voltage': pb.batt_voltage,
                'current_threshold': pb.threshold,
                'desired_threshold': pb.threshold,
                'rssi': rssi,
                'updated': datetime.datetime.now(),
                'created': datetime.datetime.now(),
                'imu': [imuData]
            }
        db.insert_one(mydict)
    else:
        if pb.HasField('status'):
            if pb.status == gb_messages_pb2.HEARTBEAT:
                # Device exists, update parameters
                newvalues = {"$set":
                        {
                            'temperature': pb.temperature,
                            'batt_voltage': pb.batt_voltage,
                            'current_threshold': pb.threshold,
                            'rssi': rssi,
                            'updated': datetime.datetime.now(),
                        }}
                db.update_one(q, newvalues)

            elif pb.status == gb_messages_pb2.IMPACT:
                prevImuData = res[0]['imu']
                prevImuData.append(imuData)
                # Device exists, update parameters
                newvalues = {"$set":
                        {
                            'temperature': pb.temperature,
                            'batt_voltage': pb.batt_voltage,
                            'current_threshold': pb.threshold,
                            'rssi': rssi,
                            'updated': datetime.datetime.now(),
                            'imu': prevImuData,
                        }}
                db.update_one(q, newvalues)

    return pb.identifier

def getCfgMsg(db, id):
    print('Attempting to find an DB entry for {}'.format(id))
    q = {"device_id": id}
    res = db.find_one(q)
    threshold = res['desired_threshold']
    print('Found desired threshold of {}'.format(threshold))
    # Create a new PB
    pb = gb_messages_pb2.LoraMsg2()
    pb.threshold = threshold
    pb.status = gb_messages_pb2.Status.HEARTBEAT
    pb.identifier = id
    return pb.SerializeToString()

def getFwBuildNum(file):
    m = re.search('app_(.*).lzma', file)
    if m is not None:
        cur_sw_version = int(m.group(1))
        return cur_sw_version
    return 0

def checkFWUpdate(build_num):
    files = os.listdir()
    for file in files:
        file_build_num = getFwBuildNum(file)
        if (file_build_num > 0) and (file_build_num > build_num):
            return True, file
    return False, ''

def sendData(updatedata, rfm9x, id):
    run = True
    address = 0
    datasize = len(updatedata)
    transfer_size = 50

    while run:
        start = time.time()
        print('Waiting for msg')
        data = rfm9x.receive(timeout=5000)
        data = bytearray()
        for i in range(100):
            data.append(0)
        rfm9x.send(data)

        if 0:
        #if data is not None:
            pb = gb_messages_pb2.LoraMsg2()
            pb.ParseFromString(data)
            if pb.identifier == id:
                #print(pb)
                if pb.HasField('reprog'):
                    address = pb.reprog.address

                pb = gb_messages_pb2.LoraMsg2()
                pb.status = gb_messages_pb2.Status.REPROGRAMMING
                pb.identifier = id
                pb.reprog.address = address
                dataToSend = min(datasize - address, transfer_size)
                pb.reprog.data = updatedata[address:address + dataToSend]
                if (address + dataToSend) == datasize:
                    pb.reprog.flags = gb_messages_pb2.Reprogramming.Flags.LAST_PACKET
                    run = False
                    print('Last packet of data')
                else:
                    pb.reprog.flags = gb_messages_pb2.Reprogramming.Flags.CONTINUE

                data = pb.SerializeToString()
                per = round((address / datasize) * 100, 1)
                rfm9x.send(data)
                stop = time.time()
                print('Updating {}% Sending {} bytes of data to address {}'.format(per, len(data), address))
                #print('Total time {}'.format(stop - start))
            else:
                print('Wrong ID')
        else:
            print('Bad data')

    print('Done programming unit')

def updateFirware(updatefile, rfm9x, id):
    timeout = 10000
    with open(updatefile, 'rb') as f:
        updatedata = f.read()

    while timeout > 0:
        time.sleep(0.01)
        timeout -= 10
        if timeout == 0:
            print('Timed out while communicating with GB')

        data = rfm9x.receive(timeout=5000)
        if data is not None:
            pb = gb_messages_pb2.LoraMsg2()
            pb.ParseFromString(data)
            print(pb)
            if pb.identifier == id:
                timeout = 10000
                print('Received data from GB')
                if pb.status != gb_messages_pb2.Status.REPROGRAMMING:
                    # Put the unit into programming mode
                    print('Putting unit into reprogramming mode')
                    pb.Clear()
                    pb.status = gb_messages_pb2.Status.REPROGRAMMING
                    pb.identifier = id
                    data = pb.SerializeToString()
                    rfm9x.send(data)
                else:
                    timeout = 0
                    sendData(updatedata, rfm9x, id)


RADIO_FREQ_MHZ = 915.0
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

rfm9x.signal_bandwidth = 125000
rfm9x.coding_rate = 4
rfm9x.spreading_factor = 7
rfm9x.enable_crc = True

myclient = pymongo.MongoClient("mongodb://localhost:27017/")
mydb = myclient['gongbuddy']
mycoll = mydb['users']

while True:
    print('Waiting for LoRa message')
    data = rfm9x.receive(timeout=5000)
    if data is not None:
        if len(data) >= 255:
            print('Big Message of {} bytes CRC errors: {}'.format(len(data), rfm9x.crc_error_count))
            print(data)
            #print('{:02x} '.format(x) for x in data)
        else:
            start = time.time()
            print('Received {} bytes'.format(len(data)))
            pb = gb_messages_pb2.LoraMsg2()
            pb.ParseFromString(data)
            print(pb)
            print("Msg from {} with rssi: {}".format(pb.identifier, rfm9x.rssi))

            # Update DB
            id = processGongData(mycoll, pb, rfm9x.rssi)
            cfg = getCfgMsg(mycoll, id)

            ret, updatefile = checkFWUpdate(pb.buildnum)
            if ret == True:
                print('Need to update firmware')
                updateFirware(updatefile, rfm9x, id)

            # Send config/ACK message
            print('Sending {} bytes'.format(len(cfg)))
            rfm9x.send(cfg)
            stop = time.time()
            print('It took {} seconds to receive and send a message'.format(stop-start))
    else:
        print('Timeout')
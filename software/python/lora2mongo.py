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
    pb = gb_messages_pb2.LoraMsg2()
    pb.ParseFromString(data)
    print("Msg from {} with rssi: {}".format(pb.identifier, rfm9x.rssi))

    # Update DB
    processGongData(mycoll, pb, rfm9x.rssi)

    # Send config/ACK message
    #rfm9x.transmit(cfg)

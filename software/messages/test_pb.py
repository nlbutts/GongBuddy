import pandas as pd
import gb_messages_pb2
import struct

def testMsg1(x):
    msg = gb_messages_pb2.LoraMsg()

    msg.buildnum = 123
    msg.status = 987
    msg.batt_voltage = 3500

    '''
    T [ms]', 'AccX [mg]', 'AccY [mg]', 'AccZ [mg]', 'GyroX [mdps]',
        'GyroY [mdps]', 'GyroZ [mdps]', 'MagX [mgauss]', 'MagY [mgauss]',
        'MagZ [mgauss]', 'P [mB]', 'T [degC]
    '''

    f = open('imu_test_data.bin', 'wb')

    for index, row in x.iterrows():
        imu = msg.imu.add()
        imu.accel.x = row['AccX [mg]']
        imu.accel.y = row['AccY [mg]']
        imu.accel.z = row['AccZ [mg]']
        imu.gyro.x  = row['GyroX [mdps]']
        imu.gyro.y  = row['GyroY [mdps]']
        imu.gyro.z  = row['GyroZ [mdps]']

        data = struct.pack('<hhhhhh',
                        imu.accel.x,
                        imu.accel.y,
                        imu.accel.z,
                        imu.gyro.x,
                        imu.gyro.y,
                        imu.gyro.z)

        f.write(data)

    f.close()

    msg.temperature = row['T [degC]']
    msg.pressure    = row['P [mB]']

    f = open('pb_test_msg.bin', 'wb')
    f.write(msg.SerializeToString())
    f.close()

def testMsg2(x):
    msg = gb_messages_pb2.LoraMsg2()
    msg.buildnum = 123
    msg.status = 987
    msg.batt_voltage = 3500
    data = bytearray()

    for index, row in x.iterrows():
        accelx = row['AccX [mg]']
        accely = row['AccY [mg]']
        accelz = row['AccZ [mg]']
        gyrox  = row['GyroX [mdps]']
        gyroy  = row['GyroY [mdps]']
        gyroz  = row['GyroZ [mdps]']

        sample = struct.pack('<hhhhhh',
                             accelx,
                             accely,
                             accelz,
                             gyrox,
                             gyroy,
                             gyroz)


        for b in sample:
            data.append(b)

    msg.imu = bytes(data)

    msg.temperature = row['T [degC]']
    msg.pressure    = row['P [mB]']

    f = open('pb_test_msg2.bin', 'wb')
    f.write(msg.SerializeToString())
    f.close()

x = pd.read_csv('gb_test_data.csv')
testMsg1(x)
testMsg2(x)
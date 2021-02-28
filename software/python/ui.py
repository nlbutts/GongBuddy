import time
import zmq
import gb_messages_pb2
import argparse

parser = argparse.ArgumentParser(description='UI for GongBuddy')
parser.add_argument('-t', '--threshold', type=int, default=1600,
                    help='Sensitivity threshold for impact detection')

args = parser.parse_args()

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://targetcam:5555")

msg = gb_messages_pb2.LoraMsg2()
msg.buildnum = 1
msg.threshold = args.threshold
gbcfg = msg.SerializeToString()

while True:
    #  Send Configuration info
    print('Sending configuration')
    socket.send(gbcfg)
    #  Wait for next request from client
    print("Waiting for data")
    message = socket.recv()
    gbdata = gb_messages_pb2.LoraMsg2()
    gbdata.ParseFromString(message)
    print(gbdata)

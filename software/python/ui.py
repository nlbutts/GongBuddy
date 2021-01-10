import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.32:5555")

while True:
    #  Send Configuration info
    print('Sending configuration')
    socket.send(b"World")
    #  Wait for next request from client
    print("Waiting for data")
    message = socket.recv()
    print("Received request: %s" % message)

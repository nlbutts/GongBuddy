import digitalio
import board
import busio
import adafruit_rfm9x
import time
import zmq

RADIO_FREQ_MHZ = 915.0
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

rfm9x.signal_bandwidth = 125000
rfm9x.coding_rate = 4
rfm9x.spreading_factor = 7
rfm9x.enable_crc = True

buf = [1, 2, 3, 4]
buf = bytes(buf)

# This is the ZMQ server
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
	print('Receiving ZMQ client message')
	cfg = socket.recv()
	print(cfg)
	print('Waiting for LoRa message')
	data = rfm9x.receive(timeout=5000)
	print("Rssi: {}".format(rfm9x.rssi))
	print('Sending data Lora data via ZMQ')
	socket.send(data)


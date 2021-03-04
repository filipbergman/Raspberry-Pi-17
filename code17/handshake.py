from multiprocessing import Process
from tuntap import TunTap
import socket
import time
import board
import busio
import digitalio
import adafruit_rfm9x
import numpy as np
import atexit
import ipaddress

RADIO_FREQ_MHZR = 868.0  # Frequency of the radio in Mhz. Must match your
RADIO_FREQ_MHZT = 869.0

CSR = digitalio.DigitalInOut(board.CE1)
RESETR = digitalio.DigitalInOut(board.D25)

CST = digitalio.DigitalInOut(board.D17)
RESETT = digitalio.DigitalInOut(board.D4)

spiR = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
spiT = busio.SPI(board.D21, MOSI=board.D20, MISO=board.D19)

rfm9xR = adafruit_rfm9x.RFM9x(spiR, CSR, RESETR, RADIO_FREQ_MHZR)
rfm9xT = adafruit_rfm9x.RFM9x(spiT, CST, RESETT, RADIO_FREQ_MHZT)

# Radio 1: Receiver
# enable CRC checking
#rfm9xR.enable_crc = True
rfm9xR.node = 3
rfm9xR.destination = 4
rfm9xR.spreading_factor = 10
#rfm9xR.signal_bandwidth = 125000
counterR = 0

# Radio 2: Transmitter NOT WORKING
# enable CRC checking
#rfm9xT.enable_crc = True
rfm9xT.node = 1
rfm9xT.destination = 2
rfm9xT.tx_power = 5
rfm9xT.spreading_factor = 10
rfm9xT.ack_retries = 1
#rfm9xT.signal_bandwidth = 125000
counterT = 0

iface= 'tun1'
tun = TunTap(nic_type="Tun", nic_name="tun1")
#tun.config(ip="192.168.2.100", mask="255.255.255.0", gateway="192.168.0.1")
tun_ip = ""

# Set up UDP tunnel
RECEIVER_IP = "192.168.0.193" # Should be receiver's IP on the local network
MY_IP = "192.168.0.170" # Should be this node's IP on the local network
UDP_PORT = 4003

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind((MY_IP, UDP_PORT))

def exit_handler():
    tun.close()

def transmit_message(message):
    tx_sock.sendto(message, (RECEIVER_IP, UDP_PORT))

def transmit():
    while True: # Checks that tun interface has an ip address
        buf = tun.read(1024)
        print("TUN BUFFER: ", buf)
        tx_sock.sendto( buf, (RECEIVER_IP, UDP_PORT))

def receive():
    while True:
        rcvd, addr = rx_sock.recvfrom(1024)
        
        if rcvd is not None:
            print("RECEIVED")
            # If ipv4 packet, write to tun interface:
            rcvd_hex = rcvd.hex()
            print("hex: ", rcvd_hex[:2])
            if rcvd_hex[:2] == "45": 
                print("WRITE TO TUN")
                tun.write(rcvd)
            elif rcvd_hex[:2] == "31":
                # if not ip packet, decode:
                rcvd = rcvd.decode()
                print("Handshake")
                handshake(rcvd[0], rcvd[1:])


def handshake(frame_type, data):
    if frame_type == "0":
        print("\nData plane\n")
    elif frame_type == "1":
        print("\nControl plane\n")
        print("my ip: ", data)
        tun_ip = ""
        for i in range(0, 4):
            tun_ip += str(int(data[i*8:(i*8)+8], 2)) # Translates each octet from bits to decimal
            if i != 3:
                tun_ip += "."
        print("IP: ", tun_ip)
        tun.config(ip=tun_ip, mask="255.255.255.0", gateway="192.168.0.1")

def sensor_value_sender():
    ip = ipaddress.IPv4Address('192.168.2.2')
    control_frame = frame(1, ip)
    control_frame_bits = control_frame.createControlFrame()
    transmit_message(control_frame_bits)
    # Send new data regularly to the database
    i = 0
    start_time = time.monotonic()
    while True:
        if time.monotonic() > start_time + 1:
            data = "tea," + str(i) + ",3.19\n"
            data_frame = frame(0, data)
            data_frame_bits = data_frame.createDataFrame()
            transmit_message(data_frame_bits)
            i += 1
            start_time = time.monotonic()

class frame:
    frame_type = 0
    data = 0
    frame = 0

    def __init__(self, frame_type, data):
        self.frame_type = frame_type
        self.data = data

    def createControlFrame(self): 
        frame = format(self.frame_type, "b")
        data_int = int(self.data)
        data_bit = format(data_int, "032b")
        print("DATA_BIT: ", data_bit)
        frame += data_bit
        return frame.encode()
    
    def createDataFrame(self): 
        frame = format(self.frame_type, "b")
        frame += self.data
        return frame.encode()
           
if __name__ == "__main__":
    atexit.register(exit_handler)
    tx_process = Process(target=transmit)
    rx_process = Process(target=receive)
    svs_process = Process(target=sensor_value_sender)

    rx_process.start()
    time.sleep(1)
    tx_process.start()
    time.sleep(1)
    svs_process.start()

    tx_process.join()
    rx_process.join()
    tun.close()
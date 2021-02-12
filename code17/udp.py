#while True:
    # Read from TUN interface
    #buf= tun.read(252)
    #print(buf)

    # Writeto TUN interface
    #tun.write(buf)

    # Close and destroyinterface
    # tun.close()
# ----------------------------------------------------------------
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

RADIO_FREQ_MHZT = 868.0  # Frequency of the radio in Mhz. Must match your
RADIO_FREQ_MHZR = 869.0

CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

CS2 = digitalio.DigitalInOut(board.D17)
RESET2 = digitalio.DigitalInOut(board.D5)

spiR = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
spiT = busio.SPI(board.D21, MOSI=board.D20, MISO=board.D19)

rfm9xR = adafruit_rfm9x.RFM9x(spiR, CS, RESET, RADIO_FREQ_MHZR)
rfm9xT = adafruit_rfm9x.RFM9x(spiT, CS2, RESET2, RADIO_FREQ_MHZT)

# Radio 1: Transmitter
# enable CRC checking
#rfm9xT.enable_crc = True
rfm9xT.node = 4
rfm9xT.destination = 3
rfm9xT.tx_power = 5
rfm9xT.spreading_factor = 10
rfm9xT.ack_retries = 1
#rfm9xT.signal_bandwidth = 125000
counterT = 0

# Radio 2: Receiver NOT WOKRING
# enable CRC checking
#rfm9xR.enable_crc = True
rfm9xR.node = 2
rfm9xR.destination = 1
rfm9xR.spreading_factor = 10
#rfm9xR.signal_bandwidth = 125000
counterR = 0


iface= 'tun0'
# Createand configurea TUN interface
tun = TunTap(nic_type="Tun", nic_name="tun0")
tun.config(ip="192.168.2.16", mask="255.255.255.0", gateway="192.168.0.1")

# Set up UDP tunnel
RECEIVER_IP = "192.168.0.193" # Should be receiver's IP on the local network
MY_IP = "192.168.0.170" # Should be this node's IP on the local network
UDP_PORT = 4001

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind((MY_IP, UDP_PORT))

def exit_handler():
    tun.close()

def transmit(counterT):
    start_transmit = time.monotonic()
    transmitted = []

    while True:
        # send a broadcast message from my_node with ID = counter
        buf = tun.read(1024)
        #print("TUN BUFFER: ", buf)
        #buffer = bytes("startup message nr {} from pi 17 node {} ".format(counterT, rfm9xT.node), "UTF-8")
        tx_sock.sendto( buf, (RECEIVER_IP, UDP_PORT))
        transmitted.append(len(buf))
        counterT = counterT + 1
        if time.monotonic() - start_transmit >= 1:
            total_time = time.monotonic() - start_transmit
            print("1 second has passed! Transmit Bitrate: {}", np.sum(transmitted)*8/total_time)
            start_transmit = time.monotonic()
            transmitted = []
        
def receive():
    start_receive = time.monotonic()
    received = []
    while True:
        rcvd, addr = rx_sock.recvfrom(1024)
        if rcvd is not None:
            print("Received (raw header):", [hex(x) for x in rcvd[0:4]])
            tun.write(rcvd)
            #print("Received (raw payload): {0}".format(rcvd[4:]))
            #print("Received RSSI: {0}".format(rfm9xR.last_rssi))
            received.append(len(rcvd))
            if time.monotonic() - start_receive >= 1:
                total_time_r = time.monotonic() - start_receive
                print("1 second has passed! Receive Bitrate: {}", np.sum(received)*8/total_time_r)
                start_receive = time.monotonic()
                received = []

if __name__ == "__main__":
    atexit.register(exit_handler)
    tx_process = Process(target=transmit, kwargs={'counterT': counterT})
    rx_process = Process(target=receive)
    
    rx_process.start()
    time.sleep(1)
    tx_process.start()
    
    tx_process.join()
    rx_process.join()
    
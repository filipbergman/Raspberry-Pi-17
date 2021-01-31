# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Example to send a packet periodically between addressed nodes
# Author: Jerry Needell
#
from multiprocessing import Process
import time
import board
import busio
import digitalio
import adafruit_rfm9x
import numpy as np

RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in Mhz. Must match your
RADIO_FREQ_MHZ2 = 915.0

CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

CS2 = digitalio.DigitalInOut(board.CE1)
RESET2 = digitalio.DigitalInOut(board.D4)

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
rfm9x2 = adafruit_rfm9x.RFM9x(spi, CS2, RESET2, RADIO_FREQ_MHZ2)

# Radio 1
rfm9x.enable_crc = True
rfm9x.node = 2
rfm9x.destination = 1
rfm9x.tx_power = 23
counter = 10

# Radio 2:
rfm9x2.enable_crc = True
rfm9x2.node = 2
rfm9x2.destination = 1
rfm9x2.tx_power = 23
counter2 = 10

def tx(radio, count, size, src, dst):
    radio.tx_power = 23

    print('Tx RFM9x sending to address: {}'.format(radio.destination))

    status = []
    buffer = np.random.bytes(size)

    start = time.monotonic()
    while count:
        result = radio.send(bytes("FFFFStartup message {} from node {}".format(counter, radio.node), "UTF-8"))
        if not result:
            status.append(False)
        else:
            print("TRANSMITTED")
            status.append(True)
        count -= 1
    total_time = time.monotonic() - start

    print('{} successfull transmissions, {} failures, {} bps'.format(sum(status), len(status)-sum(status), len(buffer)*8*len(status)/total_time))

def rx(radio, count, src):
    radio.node = src

    print('Rx RFM9x started, started at address:{}'.format(radio.node))

    received = []

    start_time = None
    start = time.monotonic()
    while count and (time.monotonic() - start) < 30:
        packet = radio.receive(timeout=5.0)
        if packet:
            print("RECEIVED")    
            print("Received (raw payload): {0}".format(packet[4:]))
            if start_time is None:
                start_time = time.monotonic()

            count -= 1
            received.append(len(packet))
        else:
            print(packet)

    total_time = time.monotonic() - start_time

    print('{} received, {} average, {} bps'.format(len(received), np.mean(received), np.sum(received)*8/total_time))


while True:
    rx_process = Process(target=rx, kwargs={'radio':rfm9x, 'count': counter, 'src': rfm9x.destination})
    tx_process = Process(target=tx, kwargs={'radio':rfm9x2, 'count': counter2, 'size': 250, 'src': rfm9x2.node, 'dst': rfm9x2.destination})    

    rx_process.start()
    time.sleep(1)
    tx_process.start()

    tx_process.join()
    rx_process.join()


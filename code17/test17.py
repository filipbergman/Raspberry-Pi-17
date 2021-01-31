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

# Define radio parameters.
RADIO_FREQ_MHZ = 868.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.
RADIO_FREQ_MHZ2 = 915.0

# Define pins connected to the chip
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

CS2 = digitalio.DigitalInOut(board.CE1)
RESET2 = digitalio.DigitalInOut(board.D4)

# Initialize SPI bus.
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
rfm9x2 = adafruit_rfm9x.RFM9x(spi, CS2, RESET2, RADIO_FREQ_MHZ2)

# Radio 1
# enable CRC checking
#rfm9x.enable_crc = True
# set node addresses
rfm9x.node = 4
rfm9x.destination = 3
rfm9x.tx_power = 23
# initialize counter
counter = 0

# Radio 2
# enable CRC checking
#rfm9x2.enable_crc = True
# set node addresses
rfm9x2.node = 2
rfm9x2.destination = 1
rfm9x2.tx_power = 23
# initialize counter
counter2 = 0

while True:
    # Start thread 1

    # Start thread 2

    # send a broadcast message from my_node with ID = counter
    rfm9x.send(bytes("startup message nr {} from radio 1 node {} ".format(counter, rfm9x.node), "UTF-8"))      
    print("Sent package...")
    counter = counter + 1;

    rfm9x2.send(bytes("startup message nr {} from radio 2 node {} ".format(counter2, rfm9x2.node), "UTF-8"))      
    print("Sent package2...")
    counter2 = counter2 + 1
    print("HEllo")
    # Look for a new packet: only accept if addresses to my_node
    packet = rfm9x.receive(with_header=True, timeout=0.5)
    packet2 = rfm9x2.receive(with_header=True, timeout=0.5)

    # If no packet was received during the timeout then None is returned.
    if packet is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print("Received (raw header):", [hex(x) for x in packet[0:4]])
        print("Received (raw payload): {0}".format(packet[4:]))
        print("Received RSSI: {0}".format(rfm9x.last_rssi))
        # send reading after any packet received
        # after 10 messages send a response to destination_node from my_node with ID = counter&0xff
        #if counter % 10 == 0:
        #    #time.sleep(0.5)  # brief delay before responding
        #    rfm9x.identifier = counter & 0xFF
        #    rfm9x.send(
        #        bytes(
        #            "message number {} from node {} ".format(counter, rfm9x.node),
        #            "UTF-8",
        #        ),
        #        keep_listening=True,
        #    )
        # If no packet was received during the timeout then None is returned.
    if packet2 is not None:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print("Received (raw header):", [hex(x) for x in packet2[0:4]])
        print("Received (raw payload): {0}".format(packet2[4:]))
        print("Received RSSI: {0}".format(rfm9x2.last_rssi))
        # send reading after any packet received
        # after 10 messages send a response to destination_node from my_node with ID$
        #if counter % 10 == 0:
        #    time.sleep(0.5)  # brief delay before responding
        #    rfm9x2.identifier = counter2 & 0xFF
        #    rfm9x2.send(
        #        bytes(
        #            "message number {} from node {} ".format(counter2, rfm9x2.node),
        #            "UTF-8",
        #        ),
        #        keep_listening=True,
        #    )

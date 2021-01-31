from multiprocessing import Process
import time
import board
import busio
import digitalio
import adafruit_rfm9x

RADIO_FREQ_MHZ = 500.0  # Frequency of the radio in Mhz. Must match your
RADIO_FREQ_MHZ2 = 900.0

CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)

CS2 = digitalio.DigitalInOut(board.CE1)
RESET2 = digitalio.DigitalInOut(board.D5)

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

rfm9xT = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
rfm9xR = adafruit_rfm9x.RFM9x(spi, CS2, RESET2, RADIO_FREQ_MHZ2)

# Radio 1: Transmitter
# enable CRC checking
#rfm9x.enable_crc = True
rfm9xT.node = 4
rfm9xT.destination = 3
rfm9xT.tx_power = 23
counterT = 0

# Radio 2: Receiver
# enable CRC checking
#rfm9x2.enable_crc = True
rfm9xR.node = 2
rfm9xR.destination = 1
rfm9xR.tx_power = 23
counterR = 0

def transmit(counterT):
    while True:
        # send a broadcast message from my_node with ID = counter
        #time.sleep(2)
        rfm9xT.send(bytes("startup message nr {} from radio 1 node {} ".format(counterT, rfm9xT.node), "UTF-8"))    
        print("Sent package...")
        counterT = counterT + 1;

def receive():
    while True:
	    print("------------------")
            # Look for a new packet: only accept if addresses to my_node
	    packet = rfm9xR.receive(with_header=True)

	    if packet is not None:
	        # Received a packet!
	        # Print out the raw bytes of the packet:
	        print("Received (raw header):", [hex(x) for x in packet[0:4]])
	        print("Received (raw payload): {0}".format(packet[4:]))
	        print("Received RSSI: {0}".format(rfm9xR.last_rssi))
	        # send reading after any packet received
	        # after 10 messages send a response to destination_node from my_node with ID$        #if counter % 10 == 0:
	        #    time.sleep(0.5)  # brief delay before responding
	        #    rfm9x2.identifier = counter2 & 0xFF
	        #    rfm9x2.send(
	        #        bytes(
	        #            "message number {} from node {} ".format(counter2, rfm9x2.node),        #            "UTF-8",
	        #        ),
	        #        keep_listening=True,
	        #    )

if __name__ == "__main__":
    tx_process = Process(target=transmit, kwargs={'counterT':counterT})
    rx_process = Process(target=receive, kwargs={})
    
    rx_process.start()
    time.sleep(1)
    tx_process.start()

    tx_process.join()
    rx_process.join()    


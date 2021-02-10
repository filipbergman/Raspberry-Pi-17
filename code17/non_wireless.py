import socket

RECEIVER_IP = "192.168.0.193" # Should be receiver's IP on the local network
MY_IP = "192.168.0.170" # Should be this node's IP on the local network
UDP_PORT = 4000

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_sock.bind((MY_IP, UDP_PORT))

tx_sock.sendto( tun.read(252), (RECEIVER_IP, UDP_PORT))

tun.read(252), addr = rx_sock.recvfrom(1024)
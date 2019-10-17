#!/usr/bin/env python3

# Good guide: https://wiki.python.org/moin/UdpCommunication

import socket

UDP_IP = "192.168.137.180"    # arduino's IP
UDP_PORT = 2390         # arduino's port
MESSAGE = b"Hello, World!"

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

# receive 
LOCAL_IP = socket.gethostname()
LOCAL_PORT = 5576
#sock.bind((LOCAL_IP, LOCAL_PORT))
print(LOCAL_IP)
while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message:", data)


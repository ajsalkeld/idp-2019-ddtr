#!/usr/bin/env python3

# Good guide: https://wiki.python.org/moin/UdpCommunication

import socket

UDP_IP = "192.168.137.180"    # arduino's IP
UDP_PORT = 2390         # arduino's port
MESSAGE = b"Hello, World!"

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

# receive 
LOCAL_IP = ''    
LOCAL_PORT = 5566
address = (LOCAL_IP, LOCAL_PORT)
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

sock.bind(address)

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message:", data)


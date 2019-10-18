#!/usr/bin/env python3

# Good guide: https://wiki.python.org/moin/UdpCommunication

import socket

UDP_IP = "192.168.43.82"    # arduino's IP
UDP_PORT = 2390         # arduino's port
MESSAGE = b"Hello, World!"

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

# receive 
LOCAL_IP = ''    
LOCAL_PORT = 8080
address = (LOCAL_IP, LOCAL_PORT)
listen = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

#listen.bind(address)

sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

#while True:
#    data, addr = listen.recvfrom(1024) # buffer size is 1024 bytes
#    print("received message:", data)


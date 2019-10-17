#!/usr/bin/env python3

# Good guide: https://wiki.python.org/moin/UdpCommunication

import socket

UDP_IP = "127.0.0.1"    # arduino's IP
UDP_PORT = 2391         # arduino's port
MESSAGE = "Hello, World!"

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
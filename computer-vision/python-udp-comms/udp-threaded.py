#!/usr/bin/env python3
# http://www.iotsharing.com/2017/06/how-to-use-udpip-with-arduino-esp32.html

import socket
import threading
import time

# bind all IP
HOST = '0.0.0.0' 
# Listen on Port 
PORT = 44445 
#Size of receive buffer   
BUFFER_SIZE = 1024    
# Create a TCP/IP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the host and port
s.bind((HOST, PORT))

def rxThread():
    s.sendto(b"Hello from python", ("192.168.43.82", 2390))
    while True:
        # Receive BUFFER_SIZE bytes data
        # data is a list with 2 elements
        # first is data
        #second is client address
        data = s.recvfrom(BUFFER_SIZE)
        if data:
            # print received data
            print('Client to Server: ' , data)
            # Message is contained in data[0]
    # Close connection
    s.close()

rxthread = threading.Thread(target=rxThread)
rxthread.start()

time.sleep(1)

s.sendto(b"stop", ("192.168.137.46", 2390))

time.sleep(0.4)

s.sendto(b"afkndsk", ("192.168.137.46", 2390))

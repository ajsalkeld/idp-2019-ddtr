#!/usr/bin/env python3

# Good guide: https://wiki.python.org/moin/UdpCommunication

import socket
import time

UDP_IP = "192.168.43.82"    # arduino's IP
UDP_PORT = 2390         # arduino's port
MESSAGE = b"Hello, World!"

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

sock.settimeout(2) #only wait 2 seconds for a resonse

# receive 
#LOCAL_IP = ''    
#LOCAL_PORT = 8080
#address = (LOCAL_IP, LOCAL_PORT)

#listen.bind(address)

while (True):
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    try:
        rec_data, addr = client_socket.recvfrom(2048) #Read response from arduino
        print(rec_data) #Print the response from Arduino
    except:
        pass

    time.sleep(1)



#while True:
#    data, addr = listen.recvfrom(1024) # buffer size is 1024 bytes
#    print("received message:", data)


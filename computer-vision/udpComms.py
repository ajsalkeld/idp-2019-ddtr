from global_stuff import *



def rxThread():
    s.sendto(b"Hello from python", (IP, 2390))
    while True:
        # Receive BUFFER_SIZE bytes data
        # data is a list with 2 elements
        # first is data
        # second is client address
        data = s.recvfrom(BUFFER_SIZE)
        if data:
            # print received data
            print('Client to Server: ' , data)
            # Message is contained in data[0]
    # Close connection
    s.close()

def sendCommand(command):
    s.sendto(command, (IP, 2390))

IP = "192.168.137.142"

# def setup():
# bind all IP
HOST = '0.0.0.0' 
# Listen on Port 
PORT = 44444 
#Size of receive buffer   
BUFFER_SIZE = 1024    
# Create a TCP/IP socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the host and port
s.bind((HOST, PORT))
# Start thread for receiving data
rxthread = threading.Thread(target=rxThread)
rxthread.start()
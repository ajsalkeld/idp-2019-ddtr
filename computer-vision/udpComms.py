from global_stuff import *
import json

CURR_UDP_DATA = {}

def rxThread():
    s.sendto(b"Hello from python", (IP, 2390))
    try:
        while True:
            global CURR_UDP_DATA
            # Receive BUFFER_SIZE bytes data
            # data is a list with 2 elements
            # first is data
            # second is client address
            new_data = s.recvfrom(BUFFER_SIZE)
            if new_data:
                new_data = "{" + new_data[0].decode() + "}"

                try:
                    parsed = json.loads(new_data)
                    
                    CURR_UDP_DATA = parsed
                    print(CURR_UDP_DATA)


                except json.decoder.JSONDecodeError:
                    print("failed to decode json: ", new_data)
            # if data:
                ## print received data
                # print('Client to Server: ' , data)
                ## Message is contained in data[0]
        # Close connection
    except Exception as e:
        print(e)
        print("stopping udp comms thread")
        sendCommand(b"stop")
        s.close()

def sendCommand(command):
    s.sendto(command, (IP, 2390))

IP = "192.168.137.110"

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

if __name__ == "__main__":
    # sendCommand(b"stop")
    rxthread = threading.Thread(target=rxThread)
    rxthread.start()
elif USE_VIDEO:
    rxthread = threading.Thread(target=rxThread)
    rxthread.start()
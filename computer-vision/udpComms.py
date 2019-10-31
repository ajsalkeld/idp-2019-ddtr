from global_stuff import *
import json

CURR_UDP_DATA = None
STOP_THREAD = False

def rxThread():
    s.sendto(b"Hello from python", (IP, 2390))
    try:
        while not STOP_THREAD:
            # Receive BUFFER_SIZE bytes data
            # data is a list with 2 elements
            # first is data
            # second is client address
            # time.sleep(0.2)

            # sendCommand(b"get status")

            # time.sleep(0.2)

            new_data = s.recvfrom(BUFFER_SIZE)
            if new_data:
                try:
                    new_data = "{" + new_data[0].decode().replace(";", ",") + "}"
                    
                    if "ACK" in new_data:
                        continue

                    try:
                        global CURR_UDP_DATA
                        parsed = json.loads(new_data)
                        # print("received:", parsed)
                        CURR_UDP_DATA = parsed


                    except json.decoder.JSONDecodeError as e:
                        print("non-json response:", new_data)
                        # pass

                except:
                    print("garbage data")
                    # pass
            else:
                print("no response")
                # pass

                    # print("failed to decode json: ", new_data, "\nreason:", e)
            # if data:
                ## print received data
                # print('Client to Server: ' , data)
                ## Message is contained in data[0]
        # Close connection
    except Exception as e:
        print("exception:", e)

    print("stopping udp comms thread")
    sendCommand(b"stop")
    s.close()

def sendCommand(command):
    s.sendto(command, (IP, 2390))

IP = "192.168.137.183"

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
    sendCommand(b"stop")
    # sendCommand(b"drop mine")
    # rxthread = threading.Thread(target=rxThread)
    # rxthread.start()
else:
    rxthread = threading.Thread(target=rxThread)

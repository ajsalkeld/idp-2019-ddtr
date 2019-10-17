#ifndef COMMS_H
#define COMMS_H

#include <WiFiNINA.h>
#include <WiFiUDP.h>
#include <SPI.h>

#define LOCALPORT 2390

int remoteIP; // For UDP. Preset these, and they'll update when packets
int remotePort = 5566; // are received.


#endif

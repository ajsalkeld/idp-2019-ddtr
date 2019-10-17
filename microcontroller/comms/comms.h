#ifndef COMMS_H
#define COMMS_H
// @ts-ignore
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <SPI.h>

#define LOCALPORT 2390

IPAddress remoteIP; // For UDP. Preset these, and they'll update when packets
int remotePort = 5566; // are received.


#endif

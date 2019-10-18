#ifndef COMMS_H
#define COMMS_H
// @ts-ignore
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <SPI.h>

#define LOCALPORT 2390

IPAddress remoteIP; // For UDP. Preset these, and they'll update when packets
unsigned int remotePort = 8080; // are received.


#endif

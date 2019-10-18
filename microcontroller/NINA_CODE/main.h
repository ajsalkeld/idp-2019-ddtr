#ifndef MAIN_H
#define MAIN_H

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define LOCALPORT 2390

IPAddress remoteIP; // For UDP. These will update when packets
unsigned int remotePort; // are received.


#endif

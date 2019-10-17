#include "comms.h"
#include "arduino_secrets.h"

#define DEBUG true

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password

int status = WL_IDLE_STATUS; // Initial Status

WiFiUDP Udp; // Adding udp class


unsigned int localPort = LOCALPORT;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back


void setup() {
  if (DEBUG) {
    Serial.begin(9600);
    while (!Serial) {
      ; // Wait for USB serial to connect
    } 
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    if (DEBUG) Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    if (DEBUG) Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    if (DEBUG) Serial.print("Attempting to connect to SSID: ");
    if (DEBUG) Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the status:
  #ifdef DEBUG
  printWifiStatus();  
  #endif

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  // If packet is received
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    remoteIP = Udp.remoteIP();
    remotePort = Udp.remotePort();
    char mess[] = "acknowledged";
    sendPacket(mess, 12);

    // Use received instruction
  }    
}

void sendPacket(char message[], int lengthMessage) {
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write(message, lengthMessage);
  Udp.endPacket();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

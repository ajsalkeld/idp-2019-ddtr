/************************************************* 
The main code for the Arduino microcontroller.
Receives commands from Python script via UDP.
*************************************************/
#include "arduino_secrets.h"
#include "main.h"

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password

int status = WL_IDLE_STATUS; // Initial Status

WiFiUDP Udp; // Adding udp class


unsigned int localPort = LOCALPORT;      // local port to listen on

char* packetBuffer; //buffer to hold incoming packet

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for USB serial to connect 
  }

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  printWifiStatus();   // Prints wifi status

  Udp.begin(localPort);
}

void loop() {
  // receive communications with commands for navigation
  // interrupt this loop to execute commands
  // and if ultrasonic detects a mine

  int packetSize = Udp.parsePacket();
  // If packet is received
  if (packetSize) {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    remoteIP = Udp.remoteIP();
    remotePort = Udp.remotePort();
    Serial.print(remoteIP);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    packetBuffer = new char[255];

    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    String command;
    command = String(command + packetBuffer);
    Serial.println(command);

    if (command == "Hello from python") {
      sendAcknowledgement(packetBuffer, packetSize);
      // Connection message received
    }
    else if (command == "stop") {
      sendAcknowledgement(packetBuffer, packetSize);
      // Action command
    }
    else {
      sendRefusal(packetBuffer, packetSize, command);
      // Message not recognised
    }

    // Delete the packetbuffer in time to receive next
    delete[] packetBuffer;
  }    
}

void sendAcknowledgement(char* packetBuffer, int packetSize) {
  // Send back acknowledgement
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write("ACK: ");
  for (int i = 0; i < packetSize; i++) {
    Udp.write(packetBuffer[i]);
  }
  Udp.endPacket();
  Serial.print("Sent acknowledgement to ");
  Serial.print(remoteIP);
  Serial.print(':');
  Serial.println(remotePort);
}

void sendRefusal(char* packetBuffer, int packetSize, String command) {
  // Send back message that NINA doesn't understand
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write("REFUSAL: ");
  for (int i = 0; i < packetSize; i++) {
    Udp.write(packetBuffer[i]);
  }
  Udp.endPacket();
  Serial.print("Sent refusal to ");
  Serial.print(remoteIP);
  Serial.print(':');
  Serial.println(remotePort);
  Serial.print("About ");
  Serial.println(command);
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

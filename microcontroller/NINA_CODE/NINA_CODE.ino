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

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Shield object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // Motor object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); // Motor object

Servo servo;  // create servo object to control a servo


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for USB serial to connect 
  }
  AFMS.begin(); // Starts with default freq
  servo.attach(9);  // attaches the servo on pin 9 to the servo object
  servo.write(130);
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

  rightMotor->setSpeed(MAX_SPEED);
  leftMotor->setSpeed(MAX_SPEED);

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
      stopMotors();
    }
    else if (command == "run until") {
      sendAcknowledgement(packetBuffer, packetSize);
      runUntilStop(NINA_FORWARDS);
    }
    else if (command == "reverse until") {
      sendAcknowledgement(packetBuffer, packetSize);
      runUntilStop(NINA_BACKWARDS);
    }
    else if (command == "turn right until") {
      sendAcknowledgement(packetBuffer,packetSize);
      runUntilStop(RIGHTWARDS);
    }
    else if (command == "turn left until") {
      sendAcknowledgement(packetBuffer,packetSize);
      runUntilStop(LEFTWARDS);
    }
    else if (command == "lift fork") {
      sendAcknowledgement(packetBuffer, packetSize);
      liftFork();
    }
    else if (command == "drop mine") {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(DROP);
      delay(30);
      runUntilStop(NINA_BACKWARDS);
      delay(500);
      liftFork();
      delay(250);
      stopMotors();
    }
    else if (command == "lower fork") {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(PICK_UP);
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

void stopMotors() {
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  delay(250);
  rightMotor->run(RELEASE);
  leftMotor->run(RELEASE);
}

void runUntilStop(int direction, int timeToRun) {
  rightMotor->setSpeed(MAX_SPEED);
  leftMotor->setSpeed(MAX_SPEED);
  switch (direction) {
    case RIGHTWARDS:
      rightMotor->run(FORWARD);
      leftMotor->run(FORWARD);
      break;
    case LEFTWARDS:
      rightMotor->run(BACKWARD);
      leftMotor->run(BACKWARD);
      break;
    case NINA_FORWARDS:
      rightMotor->run(FORWARD);
      leftMotor->run(BACKWARD);
      break;
    case NINA_BACKWARDS:
      rightMotor->run(BACKWARD);
      leftMotor->run(FORWARD);
      break;
  }
}

void liftFork() {
  for (int pos = 180; pos >= 130; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }
}

void lowerFork(int dropOrPick) {
  switch (dropOrPick) {
    case PICK_UP:
      for (int pos = 130; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(30);                       // waits 15ms for the servo to reach the position
      }
      break;
    case DROP:
      for (int pos = 130; pos <= 165; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(30);                       // waits 15ms for the servo to reach the position
      }
      break;
  }
}
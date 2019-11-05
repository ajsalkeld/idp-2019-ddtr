/************************************************* 
The main code for the Arduino microcontroller.
Receives commands from Python script via UDP.
*************************************************/
#include "arduino_secrets.h"
#include "main.h"

char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password

int status = WL_IDLE_STATUS; // Initial Status

WiFiUDP Udp; // Adding udp class

unsigned int localPort = LOCALPORT; // local port to listen on

char *packetBuffer; //buffer to hold incoming packet

int stopTimerId;
int ultrasensorId;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Shield object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);    // Motor object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);     // Motor object

Servo servo;       // create servo object to control a servo
SimpleTimer timer; // create timer object for stopping after time

void setup()
{
  Serial.begin(9600);
  //while (!Serial)
  //{
    //; // Wait for USB serial to connect
  //}
  AFMS.begin();            // Starts with default freq
  servo.attach(SERVO_PIN); // attaches the servo on pin 9 to the servo object
  servo.write(145);

  pinMode(TRIGGER_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);     // Sets the echoPin as an Input
  pinMode(HALL_PIN, INPUT);     // Sets hall pin as an input
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(AMBER_PIN, OUTPUT);
  //Taken pins: 9, 12, 13, 4, 3, 5, 6
  //Free pins; 7, 8, 10, 11
  
  digitalWrite(RED_PIN, LOW);
  digitalWrite(AMBER_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);


  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  rightMotor->setSpeed(MAX_SPEED);
  leftMotor->setSpeed(MAX_SPEED);

  printWifiStatus(); // Prints wifi status

  Udp.begin(localPort);
  ultrasensorId = timer.setInterval(125, ultrasonicChecker); // check ultrasonic 
  timer.enable(ultrasensorId);
  amberId = timer.setInterval(2000, flashAmber);
  redId = timer.setInterval(2000, flashRed);
  timer.disable(amberId);
  timer.disable(redId);
}

void loop()
{
  // receive communications with commands for navigation
  // interrupt this loop to execute commands
  // and if ultrasonic detects a mine
  timer.run();
  int packetSize = Udp.parsePacket();
  // If packet is received
  if (packetSize)
  {
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
    if (len > 0)
    {
      packetBuffer[len] = 0;
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    String command;
    int timeToRun, leftMotorSpeed, rightMotorSpeed;

    char *token;
    token = strtok(packetBuffer, del); // Get first part - command
    command = String(token);

    token = strtok(NULL, del); // Get second part - time
    if (token != NULL)
    {
      timeToRun = atoi(token);
      Serial.println(timeToRun);
      token = strtok(NULL, del);
      if (token != NULL)
      {
        leftMotorSpeed = atoi(token);
        Serial.println(leftMotorSpeed);
        token = strtok(NULL, del);
        if (token != NULL)
        {
          rightMotorSpeed = atoi(token);
          Serial.println(rightMotorSpeed);
        }
      }
    }

    if (command == "Hello from python")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      // Connection message received
    }
    else if (command == "stop")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      stopMotors();
    }
    else if ((command.indexOf("run") >= 0))
    {
      sendAcknowledgement(packetBuffer, packetSize);
      if (timeToRun >= 0)
      {
        runMotors(timeToRun, leftMotorSpeed, rightMotorSpeed);
      }
      else
      {
        runMotors(0, leftMotorSpeed, rightMotorSpeed);
      }
    }
    else if ((command.indexOf("look mines") >= 0))
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lookForMines = true;
    }
    else if ((command.indexOf("stop mines") >= 0))
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lookForMines = false;
    }
    else if (command == "lift fork")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      liftFork(PICK_UP);
    }
    else if (command == "drop mine")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(DROP);
      delay(30);
      if (liveMine)
        runMotors(2250,-MAX_SPEED, -MAX_SPEED);
      else 
        runMotors(1500,-MAX_SPEED, -MAX_SPEED);
      digitalWrite(RED_PIN, LOW);
      delay(1500);
      timer.disable(redId);
      carryingMine = false;
      liftFork(DROP);
      delay(50);
      timer.enable(ultrasensorId);
    }
    else if (command == "lower fork")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(PICK_UP);
    }
    else if (command == "shake fork")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      shakeFork();
    }
    else if (command == "check ultra")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      ultrasonicChecker();
    }
    else if (command == "check hall")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(TEST);
      Serial.println(digitalRead(HALL_PIN));
    }
    else if (command == "get status")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      sendStatus();
    }
    else if (command == "enable timer")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      timer.enable(ultrasensorId);
    }
    else if (command == "reset")
    {
      sendAcknowledgement(packetBuffer, packetSize);
      liftFork(DROP);
      carryingMine = false;
      liveMine = false;
      delay(250);
      timer.enable(ultrasensorId);
      lookForMines = false;
    }
    else
    {
      sendRefusal(packetBuffer, packetSize, command);
      // Message not recognised
    }

    // Delete the packetbuffer in time to receive next
    delete[] packetBuffer;
  }
}

void sendAcknowledgement(char *packetBuffer, int packetSize)
{
  // Send back acknowledgement
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write("ACK: ");
  for (int i = 0; i < packetSize; i++)
  {
    Udp.write(packetBuffer[i]);
  }
  Udp.endPacket();
  Serial.print("Sent acknowledgement to ");
  Serial.print(remoteIP);
  Serial.print(':');
  Serial.println(remotePort);
}

void sendRefusal(char *packetBuffer, int packetSize, String command)
{
  // Send back message that NINA doesn't understand
  Udp.beginPacket(remoteIP, remotePort);
  Udp.write("REFUSAL: ");
  for (int i = 0; i < packetSize; i++)
  {
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

void printWifiStatus()
{
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

void stopMotors()
{
  timer.disable(amberId);
  digitalWrite(AMBER_PIN, LOW);
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  delay(250);
  rightMotor->run(RELEASE);
  leftMotor->run(RELEASE);
  if (stopTimerId) timer.deleteTimer(stopTimerId);
}
// timeToRun in millieconds
void runMotors(int timeToRun, int leftMotorSpeed, int rightMotorSpeed)
{
  if (!liveMine) {
    timer.enable(amberId);
  }
  int lDirection, rDirection;
  if ((leftMotorSpeed >= 0) & (leftMotorSpeed <= 255))
  {
    leftMotor->setSpeed(leftMotorSpeed);
    lDirection = FORWARD;
  }
  else if ((leftMotorSpeed < 0) & (leftMotorSpeed >= -255))
  {
    leftMotor->setSpeed(-leftMotorSpeed);
    lDirection = BACKWARD;
  }
  if ((rightMotorSpeed >= 0) & (rightMotorSpeed <= 255))
  {
    rightMotor->setSpeed(rightMotorSpeed);
    rDirection = FORWARD;
  }
  else if ((rightMotorSpeed < 0) & (rightMotorSpeed >= -255))
  {
    rightMotor->setSpeed(-rightMotorSpeed);
    rDirection = BACKWARD;
  }
  if (timeToRun > 0)
  {
    stopTimerId = timer.setTimeout(timeToRun, stopMotors);
  }
  leftMotor->run(lDirection);
  rightMotor->run(rDirection);
}

void liftFork(int dropOrPick)
{
  /*if (pos < 125) pos = 127;
  */
  switch (dropOrPick)
  {
  case PICK_UP:
    while (pos > 125)
    {
      // in steps of 1 degree
      pos -= 1;
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(50);        // waits 15ms for the servo to reach the position
    }
    delay(100);
    forkLow = false;
    break;
  case DROP:
    pos = 125;
    servo.write(pos);
    delay(100);
    forkLow = false;
    break;
  }
}

void lowerFork(int dropOrPick)
{
  forkLow = true;
  switch (dropOrPick)
  {
  case PICK_UP:
    /*if (pos >= 165) pos = 164;
    while (pos < 165)
    { 
      // in steps of 1 degree
      pos += 1;
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(15);        // waits 15ms for the servo to reach the position
    }*/
    pos = 162;
    servo.write(pos);

    break;
  case TEST:
    /*if (pos > 150) pos = 149;
    while (pos < 150)
    { 
      // in steps of 1 degree
      pos += 1;
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(15);        // waits 15ms for the servo to reach the position
    }*/
    pos = 145;
    servo.write(pos);
    break;
  case DROP:
    /*if (pos > 160) pos = 159;
    while (pos < 160)
    { 
      // in steps of 1 degree
      pos += 1;
      servo.write(pos); // tell servo to go to position in variable 'pos'
      delay(15);        // waits 15ms for the servo to reach the position
    }*/
    pos = 152;
    servo.write(pos);
    break;
  }
}

void shakeFork()
{
  servo.write(pos-4);
  delay(250);
  servo.write(pos);
}

void ultrasonicChecker()
{
  // Note that everything is blocked in this function. Timers will not work.
  if (!carryingMine & !forkLow & lookForMines)
  {
    // Get the distance
    getUSDistance();

    if (distance < 20)  // If US detects mine is close
    {
      getUSDistance();
      if (distance < 20) {
        lookForMines = false;
        Serial.println("Close to mine");
        stopMotors();
        //getUSDistance();

        // Report back
        Udp.beginPacket(remoteIP, remotePort);
        Udp.write("Close to mine ");
        char distchar = distance;
        Udp.write(distance);
        Udp.endPacket();

        timer.disable(amberId);
        digitalWrite(AMBER_PIN, HIGH);


        // disable timer
        timer.disable(ultrasensorId);

        // Move forward over mine and check hall sensor
        // calculate time to run forward, 7.5 cm/s
        liveMine = NULL;
        if (distance > 14) {
          float timeForMine = (distance - 14) / 7.9 * 1000;
          runMotors(0, 100, 100); // Drive to 11cm away
          Serial.println((int)timeForMine);
          getUSDistance();
          lowerFork(TEST); // Lower for for hall sensor
          delay((int)timeForMine);
          stopMotors();
        }
        else {
          // reverse back...
          float timeForMine = (13 - distance) / 7.9 * 1000;
          runMotors(0, -100, -100); // Drive to 11cm away
          Serial.println((int)timeForMine);
          getUSDistance();
          lowerFork(TEST); // Lower for for hall sensor
          if((int)timeForMine > 3000) timeForMine = 3000;
          delay((int)timeForMine);
          stopMotors();
        }

        timer.disable(amberId);
        digitalWrite(AMBER_PIN, HIGH);

        // check hall sensor:
        switch (digitalRead(HALL_PIN)) {
          case LOW:
            // Not a live mine
            liveMine = false;
            digitalWrite(GREEN_PIN, HIGH);
            timeToGreenOff = millis();
            break;
          case HIGH:
            // Live mine
            liveMine = true;
            timeToGreenOff = millis();
            break;
        }
        delay(250);
        runMotors(0,-100,-100);
        delay(1000);
        stopMotors();

        //getUSDistance();
        // pickup
        lowerFork(PICK_UP);
        delay(100);
        if (distance > 30) {
          getUSDistance();
          if (distance > 30) distance = 20;
        }
        if (millis() - timeToGreenOff > 4500) {
          turnOffGreenAmber();
        }
        float timeForMine = (distance) / 7.9 * 1000 + 1000;
        runMotors(0, 100, 100);
        Serial.println((int)timeForMine);
        delay((int)timeForMine);
        stopMotors();
        liftFork(PICK_UP);
        delay(250);
        // Check US again - reattempt if not picked up
        getUSDistance();

        if (millis() - timeToGreenOff > 4500) {
          turnOffGreenAmber();
        }

        /*if (distance < 20) {
          Udp.beginPacket(remoteIP, remotePort);
          Udp.write("Failed to pickup mine, re-attempting");
          Udp.endPacket();
          carryingMine = false;
          lookForMines = true;
          timer.enable(ultrasensorId);
          return;
        }
        else {*/
        if (liveMine) timer.enable(redId);
        Udp.beginPacket(remoteIP, remotePort);
        char messageReturn[50];
        sprintf(messageReturn, "Picked up mine %d", liveMine);
        Udp.write(messageReturn);
        Udp.endPacket();
        carryingMine = true;
        if (millis() - timeToGreenOff > 4500) {
          turnOffGreenAmber();
        }
        //}
      }
    }
  }
}

void sendStatus() {
  Udp.beginPacket(remoteIP, remotePort);
  char statusReport[100];
  getUSDistance();
  sprintf(statusReport, "\"carryingMine\": %d; \"livemine\": %d; \"forkLow\": %d; \"distance\": %d; \"lookForMines\": %d", carryingMine, liveMine, forkLow, distance, lookForMines);
  Udp.write(statusReport);
  Udp.endPacket();
}

void getUSDistance() {
  // Clears the trigPin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  ultrasonicDuration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance = ultrasonicDuration * 0.034 / 2;
}

void flashAmber() {
  bool state = digitalRead(AMBER_PIN);
  switch (state) {
    case true:
      digitalWrite(AMBER_PIN, LOW);
      break;
    case false:
      digitalWrite(AMBER_PIN, HIGH);
      break;
  }
}

void flashRed() {
  bool state = digitalRead(RED_PIN);
  switch (state) {
    case true:
      digitalWrite(RED_PIN, LOW);
      break;
    case false:
      digitalWrite(RED_PIN, HIGH);
      break;
  }
}

void turnOffGreenAmber() {
  digitalWrite(AMBER_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
}

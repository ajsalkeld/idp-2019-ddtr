/************************************************* 
The main code for the Arduino microcontroller.
Receives commands from Python script via UDP.
*************************************************/
#include "arduino_secrets.h"
#include "main.h"

char ssid[] = SECRET_SSID; 	// network SSID (name)
char pass[] = SECRET_PASS; 	// network password

int status = WL_IDLE_STATUS; 	// Initial Status

unsigned int localPort = LOCALPORT; // Local port to listen on

char *packetBuffer; 		// Buffer to hold incoming packet

void setup()
{
  Serial.begin(9600);	   	// USB serial for debug
  
  AFMS.begin();            	// Starts motor shield with default freq (1600Hz)
  servo.attach(SERVO_PIN); 	// attaches the servo on pin 9 to the servo object
  servo.write(145);		// Move servo to an upward position

  // Set up digital pins
  pinMode(TRIGGER_PIN, OUTPUT); // Ultrasonic
  pinMode(ECHO_PIN, INPUT);     // 
  pinMode(HALL_PIN, INPUT);     // Hall Sensor
  pinMode(RED_PIN, OUTPUT);	// LEDs
  pinMode(GREEN_PIN, OUTPUT);	//
  pinMode(AMBER_PIN, OUTPUT);	//
  pinMode(LIVE_MINE_PIN, OUTPUT);
  pinMode(DEAD_MINE_PIN, OUTPUT);
  pinMode(SONG_PIN, OUTPUT);
  pinMode(SUCCESS_PIN, OUTPUT);

  // Set LEDs to low
  digitalWrite(RED_PIN, LOW);
  digitalWrite(AMBER_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);

  //Set sound effect pins to high (active low)  
  digitalWrite(LIVE_MINE_PIN, HIGH);
  digitalWrite(DEAD_MINE_PIN, HIGH);
  digitalWrite(SONG_PIN, HIGH);
  digitalWrite(SUCCESS_PIN, HIGH);

  // Set inital speed on motor
  rightMotor->setSpeed(MAX_SPEED);
  leftMotor->setSpeed(MAX_SPEED);

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

  printWifiStatus(); // Prints wifi status (inc IP address!)

  Udp.begin(localPort); // Start UDP comms

  // Timers
  ultrasensorId = timer.setInterval(125, ultrasonicChecker); // Begin US timer 
  timer.enable(ultrasensorId);
  // LED Timers
  amberId = timer.setInterval(2000, flashAmber);
  redId = timer.setInterval(2000, flashRed);
  timer.disable(amberId); // Disable until needed
  timer.disable(redId);   // 
}

void loop()
{
  // receive communications with commands for navigation
  // interrupt this loop to execute commands
  // and if ultrasonic detects a mine
  timer.run();	// Runs timers that are due

  int packetSize = Udp.parsePacket(); // Check for packets from Udp buffer
  // If packet is received
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    remoteIP = Udp.remoteIP();		// Updates the IP and port of the 
    remotePort = Udp.remotePort();	// computer.
    Serial.print(remoteIP);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    packetBuffer = new char[255];
    int len = Udp.read(packetBuffer, 255);
    if (len > 0)
    {
      packetBuffer[len] = 0; 		// Null character terminates char[]
    }
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    // Parse the command "run:time:left speed: right speed"
    String command;
    int timeToRun, leftMotorSpeed, rightMotorSpeed;

    char *token;			// Buffer to hold parts in
    token = strtok(packetBuffer, del); 	// Get first part - command
    command = String(token);

    token = strtok(NULL, del); 		// Get second part - time
    if (token != NULL)			// If time part doesn't exist, 
    {					// then nothing to do
      timeToRun = atoi(token);
      Serial.println(timeToRun);
      token = strtok(NULL, del);	// Get third part - left spd
      if (token != NULL)
      {
        leftMotorSpeed = atoi(token);
        Serial.println(leftMotorSpeed);
        token = strtok(NULL, del);
        if (token != NULL)
        {
          rightMotorSpeed = atoi(token); // Get fourth part - right spd
          Serial.println(rightMotorSpeed);
        }
      }
    }

    if (command == "Hello from python")	// An opening message - just prompts an ack
    {
      sendAcknowledgement(packetBuffer, packetSize);
      // Connection message received
    }
    else if (command == "stop")		// Stop the motors
    {
      sendAcknowledgement(packetBuffer, packetSize);
      stopMotors();
    }
    else if ((command.indexOf("run") >= 0))	// Our run command, as parsed above
    {
      sendAcknowledgement(packetBuffer, packetSize);
      if (timeToRun >= 0)
      {
        runMotors(timeToRun, leftMotorSpeed, rightMotorSpeed);
      }
      else
      {
        runMotors(0, leftMotorSpeed, rightMotorSpeed);	// Passing 0 runs motors indefinitely
      }
    }
    else if ((command.indexOf("look mines") >= 0))	// Begin looking for mines.
    {							// Robot will poll ultrasonic and
      sendAcknowledgement(packetBuffer, packetSize);	// stop before mines.
      lookForMines = true;				// CV calls this on approach to mine.
    }
    else if ((command.indexOf("stop mines") >= 0))	// Stop looking for mines
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lookForMines = false;
    }
    else if (command == "lift fork")			// A debug command to test the fork
    {
      sendAcknowledgement(packetBuffer, packetSize);
      liftFork(PICK_UP);
    }
    else if (command == "drop mine")			// Run the mine-dropping procedure
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(SHAKE);
      delay(300);
      liftFork(DROP);
      delay(300);
      lowerFork(SHAKE);
      delay(300);
      liftFork(DROP);
      delay(300);
      lowerFork(DROP);
      delay(300);
      if (liveMine)
        runMotors(2650,-MAX_SPEED, -MAX_SPEED);		// Live mine bin needs more reversing.
      else 
        runMotors(1900,-MAX_SPEED, -MAX_SPEED);
      timer.disable(redId);				// Stop flashing red (no longer carrying mine)
      digitalWrite(RED_PIN, LOW);			// Turn off the RED LED
      delay(1500);
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
    else if (command == "shake fork")			// Moves fork slightly to debug shaking
    {
      sendAcknowledgement(packetBuffer, packetSize);
      shakeFork();
    }
    else if (command == "check ultra")			// Updates the distance reading for debug
    {
      sendAcknowledgement(packetBuffer, packetSize);
      ultrasonicChecker();
    }
    else if (command == "check hall")			// Debug of the hall sensor
    {
      sendAcknowledgement(packetBuffer, packetSize);
      lowerFork(TEST);
      Serial.println(digitalRead(HALL_PIN));
    }
    else if (command == "get status")			// Sends current variables
    {
      sendAcknowledgement(packetBuffer, packetSize);
      sendStatus();
    }
    else if (command == "enable timer")			// Force enable the US timer for debug
    {
      sendAcknowledgement(packetBuffer, packetSize);
      timer.enable(ultrasensorId);
    }
    else if (command == "reset")			// Resets the robot
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
  static int numDrops = 0; //Variable to play song after 4 drops
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
    pos = 159;
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
    playSound(SUCCESS_PIN);
    numDrops++;
    if (numDrops == 3){
      playSound(SONG_PIN);
    }
    break;
  case SHAKE:
    pos = 168;
    servo.write(pos);
    delay(100);
    break;
  }
}

void shakeFork()
{
  servo.write(145);
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
          float timeForMine = (14 - distance) / 7.9 * 1000;
          runMotors(0, -100, -100); // Drive to 11cm away
          Serial.println((int)timeForMine);
          getUSDistance();
          lowerFork(TEST); // Lower for for hall sensor
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
            playSound(DEAD_MINE_PIN);
            timeToGreenOff = millis();
            break;
          case HIGH:
            // Live mine
            liveMine = true;
            playSound(LIVE_MINE_PIN);
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

void playSound(int sound_pin) {
  digitalWrite(sound_pin, LOW);
  delay(2);
  digitalWrite(sound_pin, HIGH);
}

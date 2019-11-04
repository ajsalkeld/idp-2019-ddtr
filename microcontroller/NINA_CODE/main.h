#ifndef MAIN_H
#define MAIN_H

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "SimpleTimer.h"


#define LOCALPORT 2390
#define MAX_SPEED 200

// Speed 100, 77cm in 10 secs = 7.7 cm/s
// Speed 150, 115cm in 10 secs = 11.5 cm/s
// Speed 200, 161cm in 10 secs = 16.1 cm/s

#define ECHO_PIN 12
#define TRIGGER_PIN 13
#define SERVO_PIN 9
#define HALL_PIN 4

#define RED_PIN 3
#define GREEN_PIN 5
#define AMBER_PIN 6

#define RIGHTWARDS 1
#define LEFTWARDS 2
#define NINA_FORWARDS 3
#define NINA_BACKWARDS 4

#define PICK_UP 1
#define DROP 2
#define TEST 3

IPAddress remoteIP; // For UDP. These will update when packets
unsigned int remotePort; // are received.

IPAddress diagIP; // For UDP. These will update when packets
unsigned int diagPort; // are received.


long ultrasonicDuration;
int distance; // cm?
bool carryingMine = false;
bool forkLow = false;
const char del[] = ":";
int pos = 130;
bool liveMine;
bool lookForMines = false;

int amberId;
int redId;
int timeToGreenOff;

void runUntilStop(int direction, int timeToRun = 0);
void runMotors(int timeToRun = 0, int leftMotorSpeed = MAX_SPEED, int rightMotorSpeed = MAX_SPEED);

//"run:time:left speed:right speed"

#endif

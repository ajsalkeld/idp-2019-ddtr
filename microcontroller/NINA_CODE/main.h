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

#define ECHO_PIN 12
#define TRIGGER_PIN 13
#define SERVO_PIN 9

#define RIGHTWARDS 1
#define LEFTWARDS 2
#define NINA_FORWARDS 3
#define NINA_BACKWARDS 4

#define PICK_UP 1
#define DROP 2

IPAddress remoteIP; // For UDP. These will update when packets
unsigned int remotePort; // are received.

long ultrasonicDuration;
int distance; // cm?
bool carryingMine = false;
const char del[] = ":";

void runUntilStop(int direction, int timeToRun = 0);
void runMotors(int timeToRun = 0, int leftMotorSpeed = MAX_SPEED, int rightMotorSpeed = MAX_SPEED);

//"run:time:left speed:right speed"

#endif

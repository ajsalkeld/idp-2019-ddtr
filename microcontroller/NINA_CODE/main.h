#ifndef MAIN_H
#define MAIN_H

// Include Libraries
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "SimpleTimer.h"


#define LOCALPORT 2390	// Port for UDP
#define MAX_SPEED 200	// Base Speed

// Speed 100, 77cm in 10 secs = 7.7 cm/s
// Speed 150, 115cm in 10 secs = 11.5 cm/s
// Speed 200, 161cm in 10 secs = 16.1 cm/s

// Digital Pins
#define ECHO_PIN 12
#define TRIGGER_PIN 13
#define SERVO_PIN 9
#define HALL_PIN 4

// LED Pins
#define RED_PIN 3
#define GREEN_PIN 5
#define AMBER_PIN 6

// Fork Lifting Variables
#define PICK_UP 1
#define DROP 2
#define TEST 3

IPAddress remoteIP; 	// For UDP. These will update when packets
unsigned int remotePort; // are received.

long ultrasonicDuration;	// Secs
int distance; 			// cm
bool carryingMine = false;
bool forkLow = false;
const char del[] = ":";		// Delimeter for run command
int pos = 130;			// Initial servo position
bool liveMine;
bool lookForMines = false;

// Timer IDs
int timeToGreenOff;
int ultrasensorId;
int stopTimerId;

// Classe and objectss
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Shield object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);    // Motor object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);     // Motor object
Servo servo;					    // servo object
SimpleTimer timer; 				    // timer object
WiFiUDP Udp; 					    // UDP object 

// Function to run motors
void runMotors(int timeToRun = 0, int leftMotorSpeed = MAX_SPEED, int rightMotorSpeed = MAX_SPEED);

// Command Structure - "run:time:left speed:right speed"

#endif

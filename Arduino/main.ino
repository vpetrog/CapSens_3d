/****************************************************************************
	Description:
		Main.ino
		Main executable for the Arduino Uno (Atmel 328p), Capacitive 3D Sensor.
		Final Project for the Sensors Technology Course
		ECE NTUA 2017
 	Authors:
		Evangelos Petrongonas 	<el14089@central.ntua.gr>
		Anastasia Rapti 	  	<el14115@central.ntua.gr>
	LICENSE: MIT
		Copyright (c) 2017 Evangelos Petrongonas, Anastasia Rapti
	Changelog:
        0.2: Changed the API, by introducing a Sensor class, which implements all the previous global functions
             like Output_Pos, Calibration, read_until etc.
		0.1: First Working Alpha
 ***************************************************************************/

// 1. INCLUDES
#include <Arduino.h>
#include "Sensor_3d.h"

// 2. CONSTANTS
#define timeout_time 600
#define AutoCal_time 60000
#define Samples 100
#define Delay 400

//Arduino Pins for the sensors
#define Sx 10
#define Rx 12
#define Sy 4
#define Ry 2
#define Sz 7
#define Rz 8

// 3. GLOBAL VARIABLES
Sensor_3d Sensor(Sx,Rx,Sy,Ry,Sz,Rz);

// 4. GLOBAL FUNCTIONS

// 5. SETUP
void setup() {
    //Initialize Serial Communication
    Serial.begin(115200);
    //Welcome Message
    Serial.println(F("Welcome to CapSens 3d, a 3D Interface Sensor."));

    //Set Timeout time for all capacitor, by uncomenting the following line
    Sensor.set_Timeout_Millis(timeout_time);

    //Set Samples to be taken for all Caps
    Sensor.set_Samples(Samples);
    //Sensor Calibration
    Sensor.Calibration();

}

// 6. MAIN LOOP
void loop() {
    Sensor.Output_Pos(uint8_t(0));
    delay(Delay);
}

// 7. FUNCTION DEFINITIONS

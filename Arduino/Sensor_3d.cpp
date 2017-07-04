/****************************************************************************
	Description:
		Sensor_3d.cpp
		Library Implementation for the Sensor using 3 CapSens_3d sensors.
		Final Project for the Sensors Technology Course
		ECE NTUA 2017
 	Authors:
		Evangelos Petrongonas 	<el14089@central.ntua.gr>
		Anastasia Rapti 	  	<el14115@central.ntua.gr>
	LICENSE: MIT
		Copyright (c) 2017 Evangelos Petrongonas, Anastasia Rapti
	Changelog:
		0.1: Implemented all the global functions of the previous Main.ino file into a new
             class. Designed the first variation of the API for the class.
 ***************************************************************************/

#include "Sensor_3d.h"
#include "Arduino.h"

Sensor_3d::Sensor_3d(uint8_t Sx,uint8_t Rx,uint8_t Sy,uint8_t Ry,uint8_t Sz,uint8_t Rz) {
    cap_x = new CapacitiveSensor(Sx,Rx); //send pin x, receive pin x
    cap_y = new CapacitiveSensor(Sy,Ry); //send pin x, receive pin x
    cap_z = new CapacitiveSensor(Sz,Rz); //send pin z, receive pin z
    start_time=0;
}

Sensor_3d::~Sensor_3d(){
    delete cap_x;
    delete cap_y;
    delete cap_z;
}

void Sensor_3d::set_Autocal_Millis(uint32_t autoCal_millis) {
    cap_x->set_CS_Autocal_Millis(autoCal_millis);
    cap_y->set_CS_Autocal_Millis(autoCal_millis);
    cap_z->set_CS_Autocal_Millis(autoCal_millis);
}

void Sensor_3d::set_Timeout_Millis(uint32_t timeout_millis){
    cap_x->set_CS_Timeout_Millis(timeout_millis);
    cap_y->set_CS_Timeout_Millis(timeout_millis);
    cap_z->set_CS_Timeout_Millis(timeout_millis);
}

void Sensor_3d::reset_Autocal() {
    cap_x->reset_CS_AutoCal();
    cap_y->reset_CS_AutoCal();
    cap_z->reset_CS_AutoCal();
}

void Sensor_3d::Calibration() {
    //uint8_t flag;
    Serial.println(F("Calibration procedure begins"));
    //Z plane calibration
    //min
    Serial.println(F("Place your hand flat above the Z plane and press enter "));
    read_until('\n');
    cap_z->set_CS_Cal_min(samples);
    //max
    Serial.println(F("Place your hand close above the Z plane and press enter (Be Carefull not to touch the Plane)"));
    read_until('\n');
    cap_z->set_Cs_Cal_max(samples);
    Serial.println(F("Plane Z calibration succesfull"));
    //Y plane Calibration
    //min
    Serial.println(F("Place your hand flat in front of the Y plane and press enter"));
    read_until('\n');
    cap_y->set_CS_Cal_min(samples);
    //max
    Serial.println(F("Place your hand close above the Y plane and press enter (Be Carefull not to touch the Plane)"));
    read_until('\n');
    cap_y->set_Cs_Cal_max(samples);
    Serial.println(F("Plane Y calibration succesfull"));
    //X plane Calibration
    //min
    Serial.println(F("Place your hand flat in front of the X plane and press enter"));
    read_until('\n');
    cap_x->set_CS_Cal_min(samples);
    //max
    Serial.println(F("Place your hand close above the X plane and press enter (Be Carefull not to touch the Plane)"));
    read_until('\n');
    cap_x->set_Cs_Cal_max(samples);
    Serial.println(F("Plane X calibration succesfull"));
    Serial.println(F("***"));

}

void Sensor_3d::Sense_3d() {
    current_x=cap_x->capacitiveSensorNormal(samples,0);
    current_y=cap_y->capacitiveSensorNormal(samples,0);
    current_z=cap_z->capacitiveSensorNormal(samples,0);
}

void Sensor_3d::set_Samples(uint8_t Samples) {
    samples= Samples;
}
void Sensor_3d::Output_Pos() {
    //Read current sensor output
    Sense_3d();
    //output time
    Serial.print(millis() - start_time);
    Serial.print("\t");

    //Ouput X plane;
    Serial.print((float)current_x,4);
    Serial.print("\t");

    //Ouput Y plane;
    Serial.print(current_y,4);
    Serial.print("\t");

    //Ouput Z plane;
    Serial.print(current_z,4);
    Serial.println();
}

void Sensor_3d::Output_Pos(uint8_t dummy) {
    Sense_3d();
    //output time
    Serial.print(millis() - start_time);
    Serial.print("\t");

    //Ouput X plane;
    Serial.print(squarify(current_x));
    Serial.print("\t");

    //Ouput Y plane;
    Serial.print(squarify(current_y));
    Serial.print("\t");

    //Ouput Z plane;
    Serial.print(squarify(current_z));
    Serial.println();
}

void Sensor_3d::Output_Pos(float dummy) {
    Sense_3d();
    //output time
    Serial.print(millis() - start_time);
    Serial.print(",");

    //Ouput X plane;
    Serial.print(squarify(current_x));
    Serial.print(",");

    //Ouput Y plane;
    Serial.print(squarify(current_y));
    Serial.print(",");

    //Ouput Z plane;
    Serial.print(squarify(current_z));
    Serial.println();
}

uint8_t Sensor_3d::read_until(const uint8_t ch) {
    uint8_t input;
    do {
        if(Serial.available()){
            input=Serial.read();
        }
    } while (input != ch);
    return input;
}

CapacitiveSensor* Sensor_3d::getCapacitiveSensor(const uint8_t axis){
    switch(axis) {
        case 'x':   return cap_x;
        case 'X':   return cap_x;
        case 'y':   return cap_y;
        case 'Y':   return cap_y;
        case 'z':   return cap_z;
        case 'Z':   return cap_z;
        default : { Serial.println("Wrong Axis Parameter"); return NULL; }
    }
}

uint8_t Sensor_3d::squarify(double value) {
    if      (value<0.015) return 200;
    else if (value<0.04) return 5;
    else if (value<0.15)  return 4;
    else if (value<0.34)  return 3;
    else if (value<0.62)  return 2;
    else if (value<1.5)  return 1;
    else return 200;
}

/****************************************************************************
	Description:
		Sensor_3d.h
        Library Declaration for the Sensor using 3 CapSens_3d sensors.
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
#ifndef SENSOR_3D_H
#define SENSOR_3D_H

#include "Arduino.h"
#include "CapSens.h"


class Sensor_3d {
public:
    Sensor_3d(uint8_t Sx, uint8_t Rx, uint8_t Sy,uint8_t Ry,uint8_t Sz,uint8_t Rz);
    void Calibration();
    void Output_Pos();         // normal values
    void Output_Pos(uint8_t);  //monitor squares
    void Output_Pos(float );   //python
    void set_Samples(uint8_t);
    uint8_t read_until(const uint8_t);
    void set_Autocal_Millis(uint32_t autoCal_millis);
    void set_Timeout_Millis(uint32_t timeout_millis);
    void reset_Autocal();
    CapacitiveSensor* getCapacitiveSensor(const uint8_t axis);
    ~Sensor_3d();
private:
    uint32_t start_time;
    uint8_t samples;
    uint32_t timeout_millis;
    double current_x,current_y,current_z;
    CapacitiveSensor* cap_x;
    CapacitiveSensor* cap_y;
    CapacitiveSensor* cap_z;
    void Sense_3d();
    uint8_t squarify(double value);
};

#endif

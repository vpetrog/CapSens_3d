/****************************************************************************
	Description:
		CapSens.cpp
		Library Implementation for the 3D Capacitive Sensor.
		Final Project for the Sensors Technology Course
		ECE NTUA 2017
 	Authors:
		Evangelos Petrongonas 	<el14089@central.ntua.gr>
		Anastasia Rapti 	  	<el14115@central.ntua.gr>
	LICENSE: MIT
		Copyright (c) 2017 Evangelos Petrongonas, Anastasia Rapti
	Note:
		The current library is a Fork of the capacitiveSensor library,
		Copyright (c) 2009 Paul Bagder
		https://github.com/PaulStoffregen/CapacitiveSensor.
		The Majority of the Library is rewritten and specifically optimized for the needs of the Project
	Changelog:
		0.2: Changed name of the library from CapSens_3d to CapSens
		0.1: First Working Alpha
 ***************************************************************************/
#include "Arduino.h"
#include "pins_arduino.h"
#include "CapSens.h"

CapacitiveSensor::CapacitiveSensor(uint8_t sendPin, uint8_t receivePin) {
	error = 1;
	loopTimingFactor = 310;		// determined empirically -  a hack
	CS_Timeout_Millis = (2000 * (float)loopTimingFactor * (float)F_CPU) / 16000000;
	CS_AutocaL_Millis = 20000;
	// get pin mapping and port for send Pin - from PinMode function in core
	#ifdef NUM_DIGITAL_PINS
		if (sendPin >= NUM_DIGITAL_PINS) error = -1;
		if (receivePin >= NUM_DIGITAL_PINS) error = -1;
	#endif
	pinMode(sendPin, OUTPUT);						// sendpin to OUTPUT
	pinMode(receivePin, INPUT);						// receivePin to INPUT
	digitalWrite(sendPin, LOW);

	sBit =  digitalPinToBitMask(sendPin);			// get send pin's ports and bitmask
	sReg = PIN_TO_BASEREG(sendPin);					// get pointer to output register
	rBit = digitalPinToBitMask(receivePin);			// get receive pin's ports and bitmask
	rReg = PIN_TO_BASEREG(receivePin);

	// get pin mapping and port for receive Pin - from digital pin functions in Wiring.c
	leastTotal = 0x0FFFFFFFL;   // input large value for autocalibrate begin
	lastCal = millis();         // set millis for start
}

int32_t CapacitiveSensor::capacitiveSensor(uint8_t samples) {

	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin

	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
	}
		// only calibrate if time is greater than CS_AutocaL_Millis and total is less than 10% of baseline
		// this is an attempt to keep from calibrating when the sensor is seeing a "touched" signal
		if ( (millis() - lastCal > CS_AutocaL_Millis) && abs(total  - leastTotal) < (int)(.10 * (float)leastTotal) ) {
			leastTotal = 0x0FFFFFFFL;          // reset for "autocalibrate"
			lastCal = millis();
		}
	// routine to subtract baseline (non-sensed capacitance) from sensor return
	if (total < leastTotal) leastTotal = total;                 // set floor value to subtract from sensed value
	return(total - leastTotal);
}

int32_t CapacitiveSensor::capacitiveSensorRaw(uint8_t samples) {
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;                  // bad pin - this appears not to work
	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
	}
	return total;
}

double CapacitiveSensor::capacitiveSensorNormal(uint8_t samples) {
	total = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin
	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
	}
	// only calibrate if time is greater than CS_AutocaL_Millis and total is less than 10% of baseline
	// this is an attempt to keep from calibrating when the sensor is seeing a "touched" signal
	if ( (millis() - lastCal > CS_AutocaL_Millis) && abs(total  - leastTotal) < (int)(.10 * (float)leastTotal) ) {
		leastTotal = 0x0FFFFFFFL;          // reset for "autocalibrate"
		lastCal = millis();
	}
	// routine to subtract baseline (non-sensed capacitance) from sensor return
	if (total < leastTotal) leastTotal = total;                 // set floor value to subtract from sensed value
	return(total - leastTotal)/maxTotal;
}

double CapacitiveSensor::capacitiveSensorNormal(uint8_t samples,uint8_t Cal) {
	total=0;
	//if ((total=capacitiveSensorRaw(samples) ) > -1) return ((total -leastTotal)/maxTotal);
	//else if (total==-2) return -2;
	for (uint8_t i = 0; i < samples; i++) {    // loop for samples parameter - simple lowpass filter
		if (SenseOneCycle() < 0)  return -2;   // variable over timeout
		if (SenseOneCycle() < 0)  return -2;
	}
	if(error<0) return -1;
	else return ((double)(total-leastTotal))/((double)(maxTotal-leastTotal));
}

void CapacitiveSensor::reset_CS_AutoCal(void){
	leastTotal = 0x0FFFFFFFL;
}

void CapacitiveSensor::set_CS_Autocal_Millis(uint32_t autoCal_millis){
	CS_AutocaL_Millis = autoCal_millis;
}

void CapacitiveSensor::set_CS_Timeout_Millis(uint32_t timeout_millis){
	CS_Timeout_Millis = (timeout_millis * (float)loopTimingFactor * (float)F_CPU) / 16000000;  // floats to deal with large numbers
}

void CapacitiveSensor::set_CS_Cal_min(uint8_t samples) {
	total=0;
	for (int i =0; i<samples; i++) {
		SenseOneCycle();
		SenseOneCycle();
	}
	//leastTotal=capacitiveSensorRaw(samples);
	CS_AutocaL_Millis=0xFFFFFFFF;
	leastTotal=total;
	Serial.println(leastTotal);
}

void CapacitiveSensor::set_Cs_Cal_max(uint8_t samples) {
	total=0;
	for (int i =0; i<samples; i++) {
		SenseOneCycle();
		SenseOneCycle();
	}
	maxTotal=total;
	//maxTotal=capacitiveSensorRaw(samples);
	CS_AutocaL_Millis=0xFFFFFFFF;
	Serial.println(maxTotal);
}

int16_t CapacitiveSensor::SenseOneCycle(void) {

  	noInterrupts();
	DIRECT_WRITE_LOW(sReg, sBit);	// sendPin Register low
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to input (pullups are off)
	DIRECT_MODE_OUTPUT(rReg, rBit); // receivePin to OUTPUT
	DIRECT_WRITE_LOW(rReg, rBit);	// pin is now LOW AND OUTPUT
	delayMicroseconds(10);
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to input (pullups are off)
	DIRECT_WRITE_HIGH(sReg, sBit);	// sendPin High
  	interrupts();

	while ( !DIRECT_READ(rReg, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is LOW AND total is positive value
		total++;
	}
	//Serial.print("SenseOneCycle(1): ");
	//Serial.println(total);
	if (total > CS_Timeout_Millis) {
		return -2;         //  total variable over timeout
	}
	// set receive pin HIGH briefly to charge up fully - because the while loop above will exit when pin is ~ 2.5V
    noInterrupts();
	DIRECT_WRITE_HIGH(rReg, rBit);
	DIRECT_MODE_OUTPUT(rReg, rBit);  // receivePin to OUTPUT - pin is now HIGH AND OUTPUT
	DIRECT_WRITE_HIGH(rReg, rBit);
	delayMicroseconds(1000);
	DIRECT_MODE_INPUT(rReg, rBit);	// receivePin to INPUT (pullup is off)
	DIRECT_WRITE_LOW(sReg, sBit);	// sendPin LOW
    interrupts();

	while ( DIRECT_READ(rReg, rBit) && (total < CS_Timeout_Millis) ) {  // while receive pin is HIGH  AND total is less than timeout
		total++;
	}

	//Serial.print("SenseOneCycle(2): ");
	//Serial.println(total);

	if (total >= CS_Timeout_Millis) {
		return -2;     // total variable over timeout
	} else {
		return 1;
	}
}

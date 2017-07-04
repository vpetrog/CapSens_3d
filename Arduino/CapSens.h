/****************************************************************************
	Description:
		CapSens.h
		Library Definition for the 3D Capacitive Sensor.
		Final Project for the Sensors Technology Course
		ECE NTUA 2017
 	Authors:
		Evangelos Petrongonas 	<el14089@central.ntua.gr>
		Anastasia Raptin 	  	<el14115@central.ntua.gr>
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

#ifndef CapSens_h
#define CapSens_h

#include "Arduino.h"

// Direct I/O through registers and bitmask (from OneWire library)
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask), (*((base)+2)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

// library interface description
class CapacitiveSensor {
public:
	 CapacitiveSensor(uint8_t sendPin, uint8_t receivePin);
	 int32_t capacitiveSensorRaw(uint8_t samples);
	 int32_t capacitiveSensor(uint8_t samples);
	 int32_t capacitiveSensor(uint8_t samples,uint8_t Cal);
	 double  capacitiveSensorNormal(uint8_t samples,uint8_t Cal);
	 double  capacitiveSensorNormal(uint8_t samples); //experimental not working
	 void 	 set_CS_Timeout_Millis(uint32_t timeout_millis);
	 void 	 reset_CS_AutoCal();
	 void 	 set_CS_Autocal_Millis(uint32_t autoCal_millis);
	 void 	 set_CS_Cal_min(uint8_t);
	 void 	 set_Cs_Cal_max(uint8_t samples);
private:
	int8_t 	  error;
    uint32_t  leastTotal;
    uint32_t  maxTotal;
	uint16_t  loopTimingFactor;
	uint32_t  CS_Timeout_Millis;
    uint32_t  CS_AutocaL_Millis;
    uint32_t  lastCal;
    uint32_t  total;
    IO_REG_TYPE sBit;   // send pin's ports and bitmask
    volatile IO_REG_TYPE *sReg;
    IO_REG_TYPE rBit;    // receive pin's ports and bitmask
    volatile IO_REG_TYPE *rReg;
    int16_t SenseOneCycle(void);
};

#endif





// X Wing Flight Controller 
// Authors: 

// Credits:
// Flight controller skeleton code borrowed from:
// dRhemFlight Flight Controller - 
//             Nicholas Rehm
//             Department of Aerospace Engineering
//             University of Maryland
//             College Park 20742
//             https://github.com/nickrehm/dRehmFlight
// -----------------------------------------------------
// Understanding of BNO055 IMU Sensor from:
// Adafruit BNO055 Absolute Orientation Sensor - 
//             Kevin Townsend
//             Adafruit
//             Last Accessed: 29/01/23
//             https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
// -----------------------------------------------------
// Understanding of GP2Y0A710K distance sensor:
//             Makerguides
//             Last Accessed: 29/01/23
//             https://www.makerguides.com/sharp-gp2y0a710k0f-ir-distance-sensor-arduino-tutorial/
// -----------------------------------------------------

// Required Libraries:
#include <Adafruit_BNO055.h> // For the BNO055 IMU sensor
#include <SharpIR.h> // For the SharpIR GP2Y0A710K distance sensor
// -----------------------------------------------------
// Pin Selection:
// NOTE: PINS PUT HERE AS PLACEHOLDERS, WILL BE CHANGED IN FINAL VERSION
// Radio:
const int ch1pin, ch2pin, ch3pin; // Don't know how many pins we'll need here
//BNO055
const int MPU;
// GP2Y0A710K
const int sharpIR;
// Motor pins:
const int m1pin, m2pin, m3pin, m4pin;
// Servo pins
const int servo1pin;
// -----------------------------------------------------


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

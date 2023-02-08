#include <SharpIR.h>


// Code to test Sharp GP2Y0A710K sensor
// Using SharpIR library

// Define model and input pin:
#define IRPin A0
#define model 100500





// Create a new instance of the SharpIR class:
SharpIR mySensor = SharpIR(IRPin, model);

void setup() {
  Serial.begin(9600);
}

void loop() {
   delay(2000);   

  // Get a distance measurement and store it as distance_cm:
  int distance_cm = mySensor.getDistance();

  // Print the measured distance to the serial monitor:
  Serial.print("Mean distance: ");
  Serial.print(distance_cm);
}

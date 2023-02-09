// Code used to test the ESC 15A. Thanks to https://bluerobotics.com/learn/controlling-basic-esc-with-the-arduino-serial-monitor/ 

#include <Servo.h>

byte servoPin = 9;
Servo servo;

void setup() {
  
 Serial.begin(9600);
 servo.attach(servoPin);

 servo.writeMicroseconds(1500); // send "stop" signal to ESC.

 delay(7000); // delay to allow the ESC to recognize the stopped signal
}


// From testing, it seems that values smaller than the 75 above the stop value (1575) will not work.
void loop() {
  
  Serial.println("Enter PWM signal value 1100 to 2200, 1500 to stop");
  
  while (Serial.available() == 0);
  
  int val = Serial.parseInt(); 
  
  if(val < 1100 || val > 2200)
  {
    Serial.println("not valid");
  }
  else
  {
    servo.writeMicroseconds(val); // Send signal to ESC.
  }
}

// Tester code for the Q Brain 4x20A Quadcopter ESC
#include <Servo.h>

// Treat ESC as 4 different ESCs
Servo ESC1, ESC2, ESC3, ESC4;

int m1pin, m2pin, m3pin,m4pin;
int testerVal = 100;
void setup() {
  ESC1.attach(m1pin);
  ESC2.attach(m2pin); 
  ESC3.attach(m3pin);
  ESC4.attach(m4pin);

  ESC1.writeMicroseconds(1500); // Send "stop" signal to ESC
  delay(7000); // delay to allow the ESC to recognize the stopped signal.
  ESC2.writeMicroseconds(1500);
  delay(7000);
  ESC3.writeMicroseconds(1500);
  delay(7000); 
  ESC4.writeMicroseconds(1500);
  delay(7000); // delay to allow the ESC to recognize the stopped signal.

}

void loop() {
  // put your main code here, to run repeatedly:
  ESC1.writeMicroseconds(testerVal);
  delay(1000);
  ESC2.writeMicroseconds(testerVal);
  delay(1000);
  ESC3.writeMicroseconds(testerVal);
  delay(1000);
  ESC4.writeMicroseconds(testerVal);
  delay(1000);
}

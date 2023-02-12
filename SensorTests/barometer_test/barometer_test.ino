// This is to test the Barometer using the Grove barometer HP20x@1.0.0 library
#include <HP20x_dev.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(150);
  /* Reset HP20x_dev */
  HP20x.begin();
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("------------------\n");
	long Temper = HP20x.ReadTemperature();
	Serial.println("Temper:");
	float t = Temper/100.0;
	Serial.print(t);	

  long Pressure = HP20x.ReadPressure();
	Serial.println("Pressure:");
	t = Pressure/100.0;
	Serial.print(t);
	Serial.println("hPa.\n");

	long Altitude = HP20x.ReadAltitude();
	Serial.println("Altitude:");
	t = Altitude/100.0;
	Serial.print(t);
	Serial.println("m.\n");

  delay(1000);  

}

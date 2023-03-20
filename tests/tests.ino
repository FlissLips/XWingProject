#include <ArduinoUnit.h> // include the ArduinoUnit library
#include <Adafruit_BNO055.h> // For the BNO055 IMU sensor
#include <barometer_dubbed.h> // Dubbed library for the barometer


#define C1 1125.0
#define C2 137500.0
Adafruit_BNO055 bno(55, 0x28, &Wire);
int distanceSensorPin = 17; 
double distanceCentimetres = 0;
unsigned char available = '0';
void ultrasonicUnit()
{
  // DESCRIPTION: Initalise ultrasonic distance sensor
  /*
   * This is added by us, and is not tested as much.
   *
   */
  Serial.println("Testing the ultrasonic distance Sensor...");
  int BUF_SIZE = 30;

  uint16_t buffer[BUF_SIZE] = {0};

  uint16_t reading = analogRead(distanceSensorPin);
  for (int i = 1; i < BUF_SIZE; i++)
    buffer[i] = buffer[i - 1];
  buffer[0] = reading;

  int avg = 0;
  for (int i = 0; i < BUF_SIZE; i++)
    avg += buffer[i];
  avg /= BUF_SIZE;
  double readingMillivolts = 1000.0 * avg * (5.0 / 1024.0);
  distanceCentimetres = 1.0 / ((readingMillivolts - C1) / C2);
  Serial.print("Measuring current (unfiltered) distance as: ");
  Serial.print(distanceCentimetres);
  Serial.println("m. \n");
  Serial.println("Please make sure this is correct.");

}

void IMUinit()
{
  // DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works.
   */
  // initializing the sensor
  Serial.println("Testing the BNO055 IMU Sensor...");
  if (!bno.begin())
  {
    // if it cannot detect the BNO055
    Serial.println("BNO055 initialization unsuccessful");
    Serial.println("Check Wiring... or maybe the I2C address..?");
  }

}


void barometerUnit()
{
  // DESCRIPTION: Initialise Barometer
  /*
   * This is added by us, and is not tested as much.
   * Also had to use our own library, since the original library.
   * didn't work on Teensy.
   */
  barometer_begin();
  delay(100);
  available = barometer_available();

  if (available != OK_HP20X_DEV)
  {
    Serial.println("Barometer does not work...");
    Serial.println("Check Wiring... or maybe the I2C address..?");
  }
}

// test(ultrasonic_test)
// {
// // Arrange
//   uint16_t expected = 100; // replace with expected distance in centimetres

//   // // Act
//   ultrasonicUnit(); // call the function being tested

//   // // Assert
//   assertEqual(expected, distanceCentimetres); // check if the distance measured by the ultrasonic sensor matches the expected value
  
// }

test(barometer_init_test)
{
  // Arrange
  unsigned char expected = OK_HP20X_DEV;
 // Act
 barometerUnit();
 //Assert
 assertEqual(expected, available);

}

test(IMU_test)
{

  int expected_bno_value = 1; // expected return value if bno.begin() succeeds
  int actual_bno_value = 0; // actual return value from bno.begin()

 // Act
  IMUinit(); // call the IMUinit function
  actual_bno_value = bno.begin(); // get the actual return value from bno.begin()

//   // Assert
  assertEqual(expected_bno_value, actual_bno_value); // check if the actual return value matches the expected value
}

void setup()
{
  Serial.begin(9600);
  while(!Serial) {} // Portability for Leonardo/Micro
}

void loop()
{
  Test::run();
}

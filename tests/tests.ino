#include <ArduinoUnit.h>      // include the ArduinoUnit library
#include <Adafruit_BNO055.h>  // For the BNO055 IMU sensor
#include <barometer_dubbed.h> // Dubbed library for the barometer

#define C1 1125.0
#define C2 137500.0
Adafruit_BNO055 bno(55, 0x28, &Wire);
int distanceSensorPin = 17;
double distanceCentimetres = 0;
unsigned char available = '0';

float thro_des, roll_des, pitch_des, yaw_des, roll_passthru, pitch_passthru, yaw_passthru;
float channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm;
float maxRoll = 30.0;  // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 30.0; // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;  // Max yaw rate in deg/sec

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

void getDesState()
{
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */

  // Needs to changed the range, as the range of our values are betweeen 171 and 1811

  int old_min = 171;
  int old_max = 1811;
  int new_min_thro = 0;
  int new_min_angle = -1;
  int new_max = 1;
 
  thro_des = ((channel_1_pwm - old_min) / (old_max - old_min)) * (new_max - new_min_thro) + new_min_thro; // Between 0 and 1
  roll_des =  ((channel_2_pwm - old_min) / (old_max - old_min)) * (new_max - new_min_angle) + new_min_angle; // Between -1 and 1
  pitch_des =  ((channel_3_pwm - old_min) / (old_max - old_min)) * (new_max - new_min_angle) + new_min_angle; // Between -1 and 1
  yaw_des = ((channel_4_pwm - old_min) / (old_max - old_min)) * (new_max - new_min_angle) + new_min_angle;   // Between -1 and 1
  roll_passthru = roll_des / 2.0;                              // Between -0.5 and 0.5
  pitch_passthru = pitch_des / 2.0;                            // Between -0.5 and 0.5
  yaw_passthru = yaw_des / 2.0;                                // Between -0.5 and 0.5  
  // Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0);    // Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll;   // Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; // Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)* maxYaw ;    // Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
 
  
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
  // Assert
  assertEqual(expected, available);
}

test(IMU_test)
{

  int expected_bno_value = 1; // expected return value if bno.begin() succeeds
  int actual_bno_value = 0;   // actual return value from bno.begin()

  // Act
  IMUinit();                      // call the IMUinit function
  actual_bno_value = bno.begin(); // get the actual return value from bno.begin()

  //   // Assert
  assertEqual(expected_bno_value, actual_bno_value); // check if the actual return value matches the expected value
}

test(getDesState_test)
{
  // Arrange
  channel_1_pwm = 991.0;
  channel_2_pwm = 991.0;
  channel_3_pwm = 991.0;
  channel_4_pwm = 991.0;

  float expected_value_thro = 0.5;
  float expected_value_roll_pitch_yaw = 0.0;
  // Act
  getDesState();

  assertEqual(thro_des, expected_value_thro);
  assertEqual(roll_des, expected_value_roll_pitch_yaw);
  assertEqual(pitch_des, expected_value_roll_pitch_yaw);
  assertEqual(yaw_des, expected_value_roll_pitch_yaw);
}
void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
  } // Portability for Leonardo/Micro
}

void loop()
{
  Test::run();
}

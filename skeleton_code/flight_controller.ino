// Flight_Controller Functions
// IMU Defines
#define ACCEL_SCALE_FACTOR 9.8
#define GYRO_SCALE_FACTOR 1
// Distance Sensor Defines
#define C1 1125.0
#define C2 137500.0
#define BUF_SIZE 30

// SETUP LOOP
// -----------------------------------------------------

// This function initalises the radio reciever
// Currently setup for SBUS communication, but can be altered
// If other radio comms are used (PPM, PWM, DSM etc.)
void commsSetup()
{
  sbus.Begin();
}
// This function initalises the BNO005 IMU Sensor
void IMUSetup()
{
  // initializing the sensor
  if (!bno.begin())
  {
    // if it cannot detect the BNO055
    Serial.println("BNO055 initialization unsuccessful");
    Serial.println("Check Wiring... or maybe the I2C address..?");
    while (1)
    {
    }
  }
}

// This function initalises the GP2Y0A710K distance sensor
// Currently, to test the sensor, it buffers 30 readings and
// takes the average (as the unfiltered readings are extremely
// inaccurate). Then, it prints the current distance to see if it's
// accurate enough.
void distanceSensorSetup()
{
  Serial.println("Currently Testing GP2Y0A710K distance sensor...");
  uint16_t buffer[BUF_SIZE] = {0};
  int counter = 0;

  uint16_t reading = analogRead(distanceSensorPin);
  for (int i = 1; i < BUF_SIZE; i++)
    buffer[i] = buffer[i - 1];
  buffer[0] = reading;

  int avg = 0;
  for (int i = 0; i < BUF_SIZE; i++)
    avg += buffer[i];
  avg /= BUF_SIZE;

  double readingMillivolts = 1000.0 * avg * (5.0 / 1024.0);
  double distanceCentimetres = 1.0 / ((readingMillivolts - C1) / C2);
  delay(1);
  if (counter++ >= 1000)
  {
    counter = 0;
    Serial.print("Measuring current (unfiltered) distance as: ");
    Serial.print(distanceCentimetres);
    Serial.println("m. \n");
    Serial.println("Please make sure this is correct.");
  }
}

// This function initalises the Grove Hi-Accuracy Barometer
void barometerSetup()
{
  Serial.println("Currently Testing Grove Hi-Accuracy Barometer...");
  HP20x.begin();
  delay(5);
  long Altitude = HP20x.ReadAltitude();
  Serial.println("(Unfiltered) Altitude:");
  t = Altitude / 100.0;
  Serial.print(t);
  Serial.println("m.\n");
  Serial.println("Please make sure this is correct.");
}

void calibrateAttitude()
{
}

// MAIN LOOP
// -----------------------------------------------------

// Reads IMU data from BNO055 IMU sensor using Adafruit library
// TO BE DONE
void getIMUdata()
{
}
// Reads Altitude data from SharpIR GP2Y0A710K distance sensor
// TO BE DONE
void getAltitudeData()
{
  // Create instance of sharp distance sensor
  SharpIR SharpIR(distanceSensorPin, model);
  // Send distance to variable
  AltSharp = SharpIR.getDistance();
}
// Normalizes desired control values to appropriate values.
// Also creates roll_passthru, pitch_passthru, and yaw_passthru variables
// for controlMixer()
// TO BE DONE
void getDesState()
{
}
// Computes control commands based on state error (angle) in cascaded scheme
//  Different control system will be used depending on the current state
//  TO BE COPIED
//  void controlSystem(int mode) {
//    // Values of
//    if (mode == 0) //Hover Mode
//    {
//      Kp_pitch_angle = 0;
//      Ki_pitch_angle = 0;
//      Kd_pitch_angle = 0;

//     Kp_roll_angle = 0;
//     Ki_roll_angle = 0;
//     Kd_roll_angle = 0;

//     Kp_roll_rate = 0;
//     Ki_roll_rate = 0;
//     Kd_roll_rate = 0;

//     Kp_pitch_rate = 0;
//     Ki_pitch_rate = 0;
//     Kd_pitch_rate = 0;

//     Kp_yaw = 0;
//     Ki_yaw= 0;
//     Kd_yaw = 0;

//   }
//   if (mode == 1)// Cruise Mode
//   {
//     Kp_pitch_angle = 0;
//     Ki_pitch_angle = 0;
//     Kd_pitch_angle = 0;

//     Kp_roll_angle = 0;
//     Ki_roll_angle = 0;
//     Kd_roll_angle = 0;

//     Kp_roll_rate = 0;
//     Ki_roll_rate = 0;
//     Kd_roll_rate = 0;

//     Kp_pitch_rate = 0;
//     Ki_pitch_rate = 0;
//     Kd_pitch_rate = 0;

//     Kp_yaw = 0;
//     Ki_yaw= 0;
//     Kd_yaw = 0;

//   }

//   }
//   if (mode == 2) // Spinning Mode
//   {
//     Kp_pitch_angle = 0;
//     Ki_pitch_angle = 0;
//     Kd_pitch_angle = 0;

//     Kp_roll_angle = 0;
//     Ki_roll_angle = 0;
//     Kd_roll_angle = 0;

//     Kp_roll_rate = 0;
//     Ki_roll_rate = 0;
//     Kd_roll_rate = 0;

//     Kp_pitch_rate = 0;
//     Ki_pitch_rate = 0;
//     Kd_pitch_rate = 0;

//     Kp_yaw = 0;
//     Ki_yaw= 0;
//     Kd_yaw = 0;

//   }
//   }
// }
// Mixing commands to motor and servo
// Different control mixing will depend on the current state
// TO BE DONE
void controlMixer(int mode)
{
}
// Computes control commands based on state error (angle) in cascaded scheme
//  TO BE DONE
void scaleCommands()
{
}

void commandOutput()
{
}
// Get values for every channel from the radio
void getCommands()
{
}

void failSafe()
{
}
// Regulate main loop rate to specified frequency in Hz
//  COPIED
void loopRate(int freq)
{
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time))
  {
    checker = micros();
  }
}
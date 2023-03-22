//Arduino/Teensy Flight Controller with extras for ASDP Project 2022-2023
//Author: Nicholas Rehm + Felicity Lipscomb
//Project Start: 1/6/2020
//Last Updated: 16/03/2023
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

*/
//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
// #define USE_PWM_RX
//#define USE_PPM_RX
#define USE_SBUS_RX
//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

//========================================================================================================================//

//REQUIRED LIBRARIES 
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <Adafruit_BNO055.h> // For the BNO055 IMU sensor
#include <SharpIR.h> // For the SharpIR GP2Y0A710K distance sensor
#include <sbus.h> // SBus Communication
#include  <barometer_dubbed.h>// For the Grove barometer HP20
#include <kalmanFilter.h> // For the Kalman filter
#include <BasicLinearAlgebra.h> // For matrix algebra
#include <ElementStorage.h> // Also for martrix algebra

//========================================================================================================================
//Setup gyro and accel full scale value selection and scale factor

#define ACCEL_SCALE_FACTOR 9.8
#define GYRO_SCALE_FACTOR 1

#define C1 1125.0
#define C2 137500.0

#define MAX_VALUE_SBUS 1811
#define MIN_VALUE_SBUS 171
#define MIN_VALUE_THROTTLE 0
#define MIN_VALUE_ANGLE -1
#define MAX_VALUE_PWM 1

// #if defined GYRO_250DPS
//   #define GYRO_SCALE GYRO_FS_SEL_250
//   #define GYRO_SCALE_FACTOR 131.0
// #elif defined GYRO_500DPS
//   #define GYRO_SCALE GYRO_FS_SEL_500
//   #define GYRO_SCALE_FACTOR 65.5
// #elif defined GYRO_1000DPS
//   #define GYRO_SCALE GYRO_FS_SEL_1000
//   #define GYRO_SCALE_FACTOR 32.8
// #elif defined GYRO_2000DPS
//   #define GYRO_SCALE GYRO_FS_SEL_2000
//   #define GYRO_SCALE_FACTOR 16.4
// #endif

// #if defined ACCEL_2G
//   #define ACCEL_SCALE ACCEL_FS_SEL_2
//   #define ACCEL_SCALE_FACTOR 16384.0
// #elif defined ACCEL_4G
//   #define ACCEL_SCALE ACCEL_FS_SEL_4
//   #define ACCEL_SCALE_FACTOR 8192.0
// #elif defined ACCEL_8G
//   #define ACCEL_SCALE ACCEL_FS_SEL_8
//   #define ACCEL_SCALE_FACTOR 4096.0
// #elif defined ACCEL_16G
//   #define ACCEL_SCALE ACCEL_FS_SEL_16
//   #define ACCEL_SCALE_FACTOR 2048.0
// #endif

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 1811; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 171; //aux1
unsigned long channel_7_fs = 171; //aux2
unsigned long channel_8_fs = 171; //rudder


// Radio failsafe values for failsafeGround()
unsigned long channel_1_fsg = 1000; //thro
unsigned long channel_2_fsg = 1500; //ail
unsigned long channel_3_fsg = 1500; //elev
unsigned long channel_4_fsg = 1500; //rudd
unsigned long channel_5_fsg = 1811; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fsg = 1811; //aux1
unsigned long channel_7_fsg = 1811; //aux2
unsigned long channel_8_fsg = 171; //rudder

// Radio failsafe values for failsafeHeight()
unsigned long channel_1_fsh = 1000; //thro
unsigned long channel_2_fsh = 1500; //ail
unsigned long channel_3_fsh = 1500; //elev
unsigned long channel_4_fsh = 1500; //rudd
unsigned long channel_5_fsh = 1811; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fsh = 1811; //aux1
unsigned long channel_7_fsh = 1811; //aux2
unsigned long channel_8_fsh = 171; //rudder

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

float length = 0.5;           // length from servoes to the centre of motors (mS)

// Altitude parameters for failsafes
float maxAlt = 100.0;
float minAlt = 0.2;
//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup

// Radio:
const int SBUSPin = 9;
// IMU Objects
const int IMUSCLPin = 19;
const int IMUSDAPin = 18;
Adafruit_BNO055 bno(55, 0x28, &Wire);
// GP2Y0A710K
const int distanceSensorPin = 17;
// Barometer
const int barometerSCL = 19;
const int barometerSDA = 18;
//OneShot125 ESC pin outputs:
const int m1Pin = 3; // Motor forward
const int m2Pin = 4; // Motor right
const int m3Pin = 5; // Motor backward
const int m4Pin = 6; // Motor left
//PWM servo or ESC outputs:
const int servo1Pin = 20;
const int servo2Pin = 21;
const int servo3Pin = 22;
const int servo4Pin = 23;
PWMServo servo1;  //Create servo objects to control a servo or ESC with PWM
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;



//========================================================================================================================//



//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
float channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm,channel_7_pwm,channel_8_pwm;
float channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

  bfs::SbusRx sbus(&Serial3);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

// Barometer
float barometerAltitude;

// Ultrasonic Distance Sensor
float ultrasonicAltitude;

// Fused altitude:
KF filter;
float fusedAltitude;
float fusedAltitudeError;
//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;


//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

// Mode:
int mode; // 0 Horizontal, 1 = Vertical, 2 = Spinning

// KILLSWITCH 0 = OFF, 1 = ON
int kill_mode;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(distanceSensorPin,INPUT);

  servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);



  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);
  Serial.println("Testing Radio Comms..");
  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
  channel_7_pwm = channel_7_fs;
  channel_8_pwm = channel_8_fs;

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  //Arm servo channels
  // servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  // servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  // servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  // servo4.write(0);
  Serial.println("IMU is successful... moving on");

  
  // delay(5);

  // calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //Arm OneShot125 motors
  m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;

  // armMotors(); //Loop over commandMotors() until ESCs happily arm
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {  // servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  // servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  // servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  // servo4.write(0);

  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  // printRadioCommands();     //Prints radio pwm values (expected: 1000 to 2000)
  // printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  // printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  // printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  // printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  //printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  //printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

  //Get vehicle state
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  getBarometerData(); //Pulls raw altitude data from barometer
  getUltrasonicData(); // Pulls raw distance data from ultrasonic sensor
  
  // Sensor fusion for attitude and altitude estimation
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  // kalmanFilter();

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits

  // Compute desired mode
  // getMode();
  //PID Controller - SELECT ONE:
  controlANGLE(); //Stabilize on angle setpoint
  // controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  //controlRATE(); //Stabilize on rate setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  // throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  // commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  // servo1.write(s1_command_PWM); //Writes PWM value to servo object
  // servo2.write(s2_command_PWM);
  // servo3.write(s3_command_PWM);
  // servo4.write(s4_command_PWM);
 
    
  //Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
  // failsafeGround(); // Prevents failures in event of being too close to the ground
  // failsafeHeight(); // Prevents failures in event of being too far away in Z-axis/Up
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

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
// How to use a SHARP GP2Y0A710K0F IR Distance Sensor - 
//             Makerguides
//             Last Accessed: 29/01/23
//             https://www.makerguides.com/sharp-gp2y0a710k0f-ir-distance-sensor-arduino-tutorial/
// -----------------------------------------------------

// Required Libraries:
#include <Adafruit_BNO055.h> // For the BNO055 IMU sensor
#include <SharpIR.h> // For the SharpIR GP2Y0A710K distance sensor
// -----------------------------------------------------
// Pin Selection
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
const int servo1pin,servo2pin,servo3pin,servo4pin;

// -----------------------------------------------------

// Global Variables
// General:
float dt;
unsigned long current_time, prev_time;
// Communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm; // Don't know how many channels needed
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev;
// IMU
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
// TODO: Need to add variables for sensor fusion? Altitude sensor fusion?
// Desired states from reciever
float roll_des, pitch_des, yaw_des, alt_des;
// Control system
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
// Control mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled;
// TODO: Add PWM values?
// -----------------------------------------------------
// User-defined Variables - to be defined during testing
// Controller parameters:
float i_limit; // Integration satuation level
float max_roll, max_pitch, max_yaw; // Max values of roll, pitch, yaw. (Need different values per mode?)

// Rotor PID
float Kp_roll_rotor, Ki_roll_rotor, Kd_roll_rotor; // PID gain values of roll
float Kp_pitch_rotor, Ki_pitch_rotor, Kd_pitch_rotor; // PID gain values of pitch
float Kp_yaw_rotor, Ki_yaw_rotor, Kd_yaw_rotor; // PID gain values of yaw

// Fixed Wing PID
float Kp_roll_fixed_wing, Ki_roll_fixed_wing, Kd_roll_fixed_wing; // PID gain values of roll
float Kp_pitch_fixed_wing, Ki_pitch_fixed_wing, Kd_pitch_fixed_wing; // PID gain values of pitch
float Kp_yaw_fixed_wing, Ki_yaw_fixed_wing, Kd_yaw_fixed_wing; // PID gain values of yaw

// Enduarance PID
float Kp_roll_endurance, Ki_roll_endurance, Kd_roll_endurance; // PID gain values of roll
float Kp_pitch_endurance, Ki_pitch_endurance, Kd_pitch_endurance; // PID gain values of pitch
float Kp_yaw_endurance, Ki_yaw_endurance, Kd_yaw_endurance; // PID gain values of yaw
// -----------------------------------------------------
void setup() {
  // put your setup code here, to run once:

}


void loop() {
  // put your main code here, to run repeatedly:

}

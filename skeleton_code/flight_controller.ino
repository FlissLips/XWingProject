// Flight_Controller Functions

#define ACCEL_SCALE_FACTOR 9.8
#define GYRO_SCALE_FACTOR 1


// SETUP LOOP
// -----------------------------------------------------

// This function initalises the radio reciever
// Currently setup for SBUS communication, but can be altered
// If other radio comms are used (PPM, PWM, DSM etc.)
void commsSetup() {
  sbus.Begin();

}
// This function initalised the BNO005 IMU Sensor
void IMUSetup() {
  //initializing the sensor
  if(!bno.begin())
  {
    //if it cannot detect the BNO055
    Serial.println("BNO055 initialization unsuccessful");
    Serial.println("Check Wiring... or maybe the I2C address..?");
    while(1){}

  }
}

void altitudeSetup() {

}

void calibrateAttitude() {

}

// MAIN LOOP
// -----------------------------------------------------

// Reads IMU data from BNO055 IMU sensor using Adafruit library
// TO BE DONE
void getIMUdata() {
  
}
// Reads Altitude data from SharpIR GP2Y0A710K distance sensor
// TO BE DONE
void getAltitudeData() {
  // Create instance of sharp distance sensor
  SharpIR SharpIR(distanceSensorPin,model);
  // Send distance to variable
  AltSharp = SharpIR.getDistance();
  
}
// Normalizes desired control values to appropriate values.
// Also creates roll_passthru, pitch_passthru, and yaw_passthru variables
// for controlMixer()
// TO BE DONE
void getDesState() {

}
//Computes control commands based on state error (angle) in cascaded scheme
// Different control system will be used depending on the current state
// TO BE COPIED
// void controlSystem(int mode) {
//   // Values of 
//   if (mode == 0) //Hover Mode
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
void controlMixer(int mode) {
 
}
//Computes control commands based on state error (angle) in cascaded scheme
// TO BE DONE
void scaleCommands() {

  
}

void commandOutput() {

}
//Get values for every channel from the radio
void getCommands() {

}

void failSafe() {

}
//Regulate main loop rate to specified frequency in Hz
// COPIED
void loopRate(int freq) {
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//initialize IMU 

void IMUinit(){

  //setting delay between fresh samples
  uint16_t IMU_SAMPLERATE_DELAY_MS=100;
  //check device address ?
void setup(void)
 {
  Serial.begin(115200);
  while (!Serial) delay(10);  //wait for serial port to open
  Serial.println("Orientation Sensor Test"); Serial.println("");

  //initializing the sensor
  if(!bno.begin())
  {
    //if it cannot detect the BNO055
    Serial.print("BNO055 initialization unsuccessful");
    while(1);
  }
  
  delay(1000);
 }



//
void getImuData();
//
  //int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  
  sensor_event_t orientationData,angVelocityData, linearAccelData,magnetometerData,accelerometerData,gravityData;

  
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

 

 //Accelerometer
 //dividing by sclae factor G
 AccX=linearAccelData.x/ACCEL_SCALE_FACTOR;
 AccY=linearAccelData.y/ACCEL_SCALE_FACTOR;
 AccZ=linearAccelData.z/ACCEL_SCALE_FACTOR;
//correct the outputs with the caluclated error values
AccX=AccX- AccErrorX;
AccY=AccY- AccErrorY;
AccZ=AccZ - AccErrorZ;
//next step filtering...

//Gyro
//scale factor deg/sec
GyroX=orientationData.x/GYRO_SCALE_FACTOR;
GyroY=orientationData.y/GYRO_SCALE_FACTOR;
GyroZ=orientationData.z/GYRO_SCALE_FACTOR;
//subtract error values
GyroX=GyroX- GyroErrorX;
GyroY=GyroY-GyroErrorY;
GyroZ=GyroZ - GyroErrorZ;
//filter...

//Magnetometer
//the scale factor: 6 uT 
MagX=magnetometerData.x/6.00;
MagY=magnetometerData.y/6.00;
MagZ=magnetometerData.z/6.00;
//subtract error
MagX=(MagX-MagErrorX)*MagScaleX;
MagY=(MagY-MagErrorY)*MagScaleY;
MagZ=(MagZ - MagErrorZ)*MagScaleZ;

//filtering..

}

//void calculate_IMU_error()

  


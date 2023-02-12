#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>
//initialize IMU 

#define ACCEL_SCALE_FACTOR 9.8
#define GYRO_SCALE_FACTOR 1

//setting delay between fresh samples
uint16_t IMU_SAMPLERATE_DELAY_MS=100;
Adafruit_BNO055 bno(55, 0x28, &Wire);
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
void getImuData()
{
  //
    //int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
    
  sensors_event_t orientationData,angVelocityData, linearAccelData,magnetometerData,accelerometerData,gravityData;

  
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  

  //Accelerometer
  //dividing by sclae factor G
  double  AccX=linearAccelData.acceleration.x/ACCEL_SCALE_FACTOR;
  double AccY=linearAccelData.acceleration.y/ACCEL_SCALE_FACTOR;
  double AccZ=linearAccelData.acceleration.z/ACCEL_SCALE_FACTOR;
  //next step filtering...

  //Gyro
  //scale factor deg/sec
  double GyroX=orientationData.gyro.x/GYRO_SCALE_FACTOR;
  double GyroY=orientationData.gyro.y/GYRO_SCALE_FACTOR;
  double GyroZ=orientationData.gyro.z/GYRO_SCALE_FACTOR;
  //filter...

  //Magnetometer
  //the scale factor: 6 uT 
  double MagX=magnetometerData.magnetic.x/6.00;
  double MagY=magnetometerData.magnetic.y/6.00;
  double MagZ=magnetometerData.magnetic.z/6.00;

  // char buffer[1024];
  // sprintf(
  //   buffer,
  //   "Acc: (%lf, %lf, %lf)\n"
  //   "Gyro: (%lf, %lf, %lf)\n"
  //   "Mag: (%lf, %lf, %lf)\n\0",
  //   AccX, AccY, AccZ,
  //   GyroX, GyroY, GyroZ,
  //   MagX, MagY, MagZ
  // );
  // Serial.print(buffer);
  Serial.print("Acceleration: ");  
  Serial.println(AccX);
  Serial.println(AccY);
  Serial.println(AccZ);
  Serial.print("Magnetometer: ");  
  Serial.println(MagX);
  Serial.println(MagY);
  Serial.println(MagZ);
  Serial.print("Orientation: ");  
  Serial.println(GyroX);
  Serial.println(GyroY);
  Serial.println(GyroZ);
  delay(1000);


//filtering..
}

void loop()
{
  getImuData();
}

//void calculate_IMU_error()

  


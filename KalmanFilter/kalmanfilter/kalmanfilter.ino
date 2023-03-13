
#include "Arduino.h"
#include <Wire.h> // I2C Communication
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
// #include <HP20x_dev.h>
#include <Adafruit_BNO055.h> // For the BNO055 IMU sensor
#include <barometer_dubbed.h>

#define C1 1125.0
#define C2 137500.0
#define ACCEL_SCALE_FACTOR 9.8
#define pinNo 17

class KF {
  public: 
    BLA::Matrix<3> state = {1,1,1};
    BLA::Matrix<3,3> covariance = {1,0,0,0,1,0,0,0,1};
    BLA::Matrix<3,3> process_noise = {10E-3,0,0,0,10E-6,0,0,0,10E-3};
  
  KF() = default;
  ~KF() = default;
  BLA::Matrix<3,3> state_transition(float dt);
  void update_step(BLA::Matrix<1,3> observation,BLA::Matrix<1> measurement ,BLA::Matrix<1> measurement_covariance);
  void prediction_step(float dt);
  BLA::Matrix<3> kalman_gain(BLA::Matrix<3,3> covariance, BLA::Matrix<1,3> observation,BLA::Matrix<1> measurement_covariance);
  BLA::Matrix<3> state_extrapolation(BLA::Matrix<3> state, BLA::Matrix<3,3> state_transition);
  BLA::Matrix<3,3> covariance_extrapolation(BLA::Matrix<3,3> covariance, BLA::Matrix<3,3> state_transition, BLA::Matrix<3,3> process_noise);
  BLA::Matrix<3> state_update(BLA::Matrix<3> state, BLA::Matrix<3> kalman_gain, BLA::Matrix<1,3> observation, BLA::Matrix<1> measurement);
  BLA::Matrix<3,3> covariance_update(BLA::Matrix<3,3> covariance,BLA::Matrix<3> kalman_gain,BLA::Matrix<1,3> observation,BLA::Matrix<1> measurement_covariance);
  };


BLA::Matrix<3,3> KF::state_transition(float dt)
{
  float a = dt;
  float b = 0.5*(dt*dt);
  BLA::Matrix<3,3> state_transition = {1.0,0,0,a,1.0,0,b,a,1.0};
  return state_transition;
}

BLA::Matrix<3> KF::state_update(BLA::Matrix<3> state, BLA::Matrix<3> kalman_gain, BLA::Matrix<1,3> observation, BLA::Matrix<1> measurement)
{
 BLA::Matrix<3> x = state;
 BLA::Matrix<3> k = kalman_gain;
 BLA::Matrix<1,3> z = observation;
 BLA::Matrix<1> h = measurement;
 BLA::Matrix<3> new_state = x + k * (h - z * x);
 return new_state;
}

BLA::Matrix<3,3> KF::covariance_update(BLA::Matrix<3,3> covariance, BLA::Matrix<3> kalman_gain, BLA::Matrix<1,3> observation, BLA::Matrix<1> measurement_covariance)
{
  BLA::Matrix<3, 3> i = {1,0,0,0,1,0,0,0,1};
  BLA::Matrix<3> k = kalman_gain;
  BLA::Matrix<1,3> kt = ~kalman_gain;
  BLA::Matrix<3,3> p = covariance;
  BLA::Matrix<1,3> h = observation;
  BLA::Matrix<1> r = measurement_covariance;
  BLA::Matrix<3,3> ikh_inv = ~(i - k*h);
  BLA::Matrix<3,3> new_covariance = (i - k * h)*p*ikh_inv+k *r*kt;
  return new_covariance;
}

BLA::Matrix<3> KF::kalman_gain(BLA::Matrix<3,3> covariance, BLA::Matrix<1,3> observation, BLA::Matrix<1> measurement_covariance)
{
  BLA::Matrix<3,3> p = covariance;
  BLA::Matrix<1,3> h = observation;
  BLA::Matrix<3> ht = ~observation;
  BLA::Matrix<1> r = measurement_covariance;
  BLA::Matrix<1> hphtr_inv;
  BLA::Invert(h*p*ht+r,hphtr_inv);
  BLA::Matrix<3> new_kalman_gain = p * ht * hphtr_inv;
  return new_kalman_gain;
}

void KF::update_step(BLA::Matrix<1,3> observation, BLA::Matrix<1> measurement, BLA::Matrix<1> measurement_covariance)
{
  BLA::Matrix<3> kalman_gain = KF::kalman_gain(KF::covariance, observation, measurement_covariance);
  BLA::Matrix<3> new_state = KF::state_update(KF::state, kalman_gain, observation, measurement);
  BLA::Matrix<3,3> new_covariance = KF::covariance_update(KF::covariance,  kalman_gain, observation, measurement_covariance);
  state = new_state;
  covariance = new_covariance;    
}


void KF::prediction_step(float dt)
{
  BLA::Matrix<3,3> state_transition = KF::state_transition(dt);
  BLA::Matrix<3> new_state = KF::state_extrapolation(KF::state, state_transition);
  BLA::Matrix<3,3> new_covariance = KF::covariance_extrapolation(covariance, state_transition, KF::process_noise);
  state = new_state;
  covariance = new_covariance;

}



BLA::Matrix<3> KF::state_extrapolation(BLA::Matrix<3> state, BLA::Matrix<3, 3> state_transition)
{
  BLA::Matrix<3, 3> f = state_transition;
  BLA::Matrix<3> x = state;
  BLA::Matrix<3> new_state = f * x;
  return new_state;
}

BLA::Matrix<3,3> KF::covariance_extrapolation(BLA::Matrix<3, 3> covariance, BLA::Matrix<3, 3> state_transition, BLA::Matrix<3, 3> process_noise)
{
  BLA::Matrix<3, 3> p = covariance;
  BLA::Matrix<3, 3> f = state_transition;
  BLA::Matrix<3, 3> ft = ~state_transition;  
  BLA::Matrix<3, 3> q = process_noise;

  BLA::Matrix<3,3> new_covariance = f * p * ft + q;
  return new_covariance;
}


Adafruit_BNO055 bno(55, 0x28, &Wire);
unsigned long lastUpdate;
KF filter;
void setup(){
// Setup IMU
// initializing the sensor

  Serial.println("Currently Testing IMU...");
  if (!bno.begin())
  {
    // if it cannot detect the BNO055
    Serial.println("BNO055 initialization unsuccessful");
    Serial.println("Check Wiring... or maybe the I2C address..?");
    while (1)
    {
    }
  }
  Serial.println("IMU Sucessful... \n Currently Testing Grove Hi-Accuracy Barometer...");
  barometer_begin();
  Serial.println("Barometer successful... \n Testing Distance Sensor");
  pinMode(pinNo,INPUT);

  lastUpdate = millis();

  
  
}

void loop(){
  float dt = (millis()-lastUpdate)/1E3;
  lastUpdate = millis();

  BLA::Matrix<1,3> observation_matrix_position = {0,0,1};
  BLA::Matrix<1> measurement_noise_barometer = {100};
  BLA::Matrix<1> measurment_noise_dist_snr = {10};
  BLA::Matrix<1,3> observation_matrix_acc = {1,0,0};
  BLA::Matrix<1> measurement_noise_acc = {5};

  unsigned long altitude = barometer_read_altitude()/100;
  BLA::Matrix<1> measurement_barometer = {altitude};

  filter.prediction_step(dt);
  filter.update_step(observation_matrix_position, measurement_barometer, measurement_noise_barometer);
 
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  double AccZ=linearAccelData.acceleration.z/ACCEL_SCALE_FACTOR;
  BLA::Matrix<1> measurement_acc = {AccZ};  

  filter.update_step(observation_matrix_acc, measurement_acc, measurement_noise_acc);

  uint16_t distance_sensor = analogRead(pinNo);
  BLA::Matrix<1> measurement_dist_snr = {distance_sensor};
  filter.update_step(observation_matrix_position, measurement_dist_snr, measurment_noise_dist_snr);


  Serial.print("Original Accelerometer: ");
  Serial.println(AccZ);
  Serial.print("Original Barometer: ");
  Serial.println(altitude);
  Serial.print("Original Distance Sensor: ");
  Serial.println(distance_sensor);
  Serial.print("Filtered Altitude: ");
  Serial.println(filter.state(2));
  Serial.print("Covariance: ");
  Serial.println(filter.covariance(2,2));

  delay(500);
  
  unsigned char ret = barometer_available();
  if(ret == OK_HP20X_DEV)
    Serial.println("bar ok");
  else
    Serial.println("bar bad");

}

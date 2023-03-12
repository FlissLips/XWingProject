#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

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
  KF::state = new_state;
  KF::covariance = new_covariance;    
}


void KF::prediction_step(float dt)
{
  BLA::Matrix<3,3> state_transition = KF::state_transition(dt);
  BLA::Matrix<3> new_state = KF::state_extrapolation(KF::state, state_transition);
  BLA::Matrix<3,3> new_covariance = KF::covariance_extrapolation(covariance, state_transition, KF::process_noise);
  KF::state = new_state;
  KF::covariance = new_covariance;

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



void setup(){

}

void loop(){

}
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

#ifndef _KALMAN_FILTER
#define _KALMAN_FILTER

class KF
{
public:
    BLA::Matrix<3> state;
    BLA::Matrix<3, 3> covariance;
    BLA::Matrix<3, 3> process_noise;

    KF() = default;
    ~KF() = default;

    // Creates State Transition Matrix
    BLA::Matrix<3, 3> state_transition(float dt);
    // Update Step of the Kalman filter. Updates the state based on the measurements, measurment covariance and the observation matrix.
    void update_step(BLA::Matrix<1, 3> observation, BLA::Matrix<1> measurement, BLA::Matrix<1> measurement_covariance);
    // Prediction Step of the Kalman filter. Predicts the state between update steps.
    void prediction_step(float dt);
    // Computes the Kalman Gain
    BLA::Matrix<3> kalman_gain(BLA::Matrix<3, 3> covariance, BLA::Matrix<1, 3> observation, BLA::Matrix<1> measurement_covariance);
    // Extrapolates the states
    BLA::Matrix<3> state_extrapolation(BLA::Matrix<3> state, BLA::Matrix<3, 3> state_transition);
    // Extrapolates the covariances
    BLA::Matrix<3, 3> covariance_extrapolation(BLA::Matrix<3, 3> covariance, BLA::Matrix<3, 3> state_transition, BLA::Matrix<3, 3> process_noise);
    // Updates the state
    BLA::Matrix<3> state_update(BLA::Matrix<3> state, BLA::Matrix<3> kalman_gain, BLA::Matrix<1, 3> observation, BLA::Matrix<1> measurement);
    // Updates the covariances
    BLA::Matrix<3, 3> covariance_update(BLA::Matrix<3, 3> covariance, BLA::Matrix<3> kalman_gain, BLA::Matrix<1, 3> observation, BLA::Matrix<1> measurement_covariance);
};

#endif
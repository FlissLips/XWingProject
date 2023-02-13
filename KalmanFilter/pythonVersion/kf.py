# Kalman Filter code
import numpy as np
class KF:
    def __init__(self):
        self.state  = np.reshape([1,1,1], (3,1))
        self.covariance = np.eye(3);
        self.process_noise
    
    def state_transition(self,dt):
        a = dt
        b = 0.5*((dt*dt)**2)
        state_transition = np.matrix(
            [
                [1,0,0],
                [a,1,0]
                [b,a,1]
            ]
        )
        return state_transition

    def update_step(self, observation, measurement, measurement_covariance):
        kalman_gain = self.kalman_gain(self.covariance, observation, measurement_covariance)
        new_state = self.state_update(self.state, kalman_gain, observation, measurement)
        new_covariance = self.covariance_update(self.covariance, kalman_gain, observation, measurement_covariance)
        self.state = new_state
        self.covariance = new_covariance

    def prediction_step(self, delta_time):
        state_transition = self.state_transition(delta_time)
        new_state = self.state_extrapolation(self.state, state_transition)
        new_covariance = self.covariance_extrapolation(self.covariance, state_transition, self.process_noise)
        self.state = new_state
        self.covariance = new_covariance

    def kalman_gain(self, covariance, observation, measurement_covariance):
        p = covariance
        h = observation
        ht = np.transpose(observation)
        r = measurement_covariance
        kalman_gain = p * ht * np.linalg.inv(h * p * ht + r)
        return kalman_gain

    def state_extrapolation(self, state, state_transition):
        f = state_transition
        x = state
        new_state = f * x
        return new_state

    def covariance_extrapolation(self, covariance, state_transition, process_noise):
        p = covariance
        f = state_transition
        ft = np.transpose(state_transition)
        q = process_noise
        new_covariance = f * p * ft + q
        return new_covariance

    def state_update(self, state, kalman_gain, observation, measurement):
        x = state
        k = kalman_gain
        z = measurement
        h = observation
        new_state = x + k * (measurement - observation * state)
        return new_state

    def covariance_update(self, covariance, kalman_gain, observation, measurement_covariance):
        i = np.identity(covariance.shape[0], covariance.dtype)
        k = kalman_gain
        kt = np.transpose(k)
        h = observation
        p = covariance
        r = measurement_covariance
        new_covariance = (i - k * h)*p*np.transpose(i - k * h) + k * r * kt
        return new_covariance
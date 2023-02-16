from .sensor import Sensor
import numpy as np
import math

class Drone:
    def __init__(self):
        self.position = 0
        self.velocity = 0
        self.acceleration = 0

        self.speed_factor = 1

        noise_mean = 0.05
        noise_variance= 0.03*0.03
        self._imu = Sensor(noise_mean,noise_variance)

        noise_mean = 0
        noise_variance= 10
        self._barometer = Sensor(noise_mean,noise_variance)

        noise_mean = 0.02
        noise_variance = 0.01
        self._distance_sensor = Sensor(noise_mean,noise_variance)
    
    def update(self, time):

        s = self.speed_factor
        t = time
        A = 10
        omega = 0.5
        position = A * math.cos(0.7 * t)
        velocity = - A * omega * math.sin(0.7*t)
        acceleration = - A * (omega * omega) * math.cos(0.7*t)

        self.position, self.velocity, self.acceleration = position, velocity, acceleration

    def measure_acceleration(self):
        measurement = self._imu.measure(self.acceleration)
        return measurement

    def measure_position_bar(self):
        measurement = self._barometer.measure(self.position)
        return measurement   
    
    def measure_position_dis(self):
        measurement = self._distance_sensor.measure(self.position)
        return measurement   

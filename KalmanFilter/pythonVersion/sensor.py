from random import gauss
import numpy as np

def gaussian_noise(mean, variance):
    return gauss(mean, variance**0.5)

class Sensor:
    def __init__(self, noise_mean, noise_variance):
        self.noise_mean = noise_mean
        self.noise_variance = noise_variance
        
    def measure(self, truth):
        measurement = truth + gaussian_noise(self.noise_mean, self.noise_variance)
        covariance = self.noise_variance

        return measurement, covariance


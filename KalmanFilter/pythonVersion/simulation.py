import numpy as np
import time
import pythonVersion as setup


class Simulation:
    def __init__(self):
        self.drone = setup.Drone()
        self.filter = setup.KF()

        self.start_time = time.time()
        self.sim_time = 0
        self.last_update = time.time()

        self.last_imu = 0
        self.imu_delay = 100
        self.last_barometer = 0
        self.barometer_delay = 0.0005
        self.last_distance_sensor = 0
        self.distance_sensor_delay = 10

    def step(self):

        t = time.time()
        dt = t - self.last_update

        self.update_kf(dt)
        self.update_drone()

        self.sim_time += dt
        self.last_update = t

        

    def update_drone(self):
        self.drone.update(self.sim_time)

    def update_kf(self,dt):
        if self.sim_time - self.last_imu >= self.imu_delay:
            self.last_imu = self.sim_time
            measurement, measurement_covariance = self.drone.measure_acceleration()
            observation  = np.matrix ([1,0,0])
            self.filter.update_step(observation, measurement,measurement_covariance)

        if self.sim_time - self.last_barometer >= self.barometer_delay:
            self.last_barometer = self.sim_time
            measurement, measurement_covariance = self.drone.measure_position_bar()
            observation = np.matrix([0,0,1])
            self.filter.update_step(observation, measurement,measurement_covariance)

        if self.sim_time - self.last_distance_sensor >= self.distance_sensor_delay:
            self.last_distance_sensor = self.sim_time
            measurement, measurement_covariance = self.drone.measure_position_dis()
            observation = np.matrix([0,0,1])
            self.filter.update_step(observation, measurement,measurement_covariance)


        self.filter.prediction_step(dt)
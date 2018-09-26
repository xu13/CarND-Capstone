import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.2
        self.throttle_control = PID(kp, ki, kd, min_throttle, max_throttle)

        tau = 0.5  # 1/(2pi*tau) = cut off frequency
        ts = 0.02  # sample time
        self.velocity_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled:
            self.throttle_control.reset()
            return 0.0, 0.0, 0.0

        current_velocity = self.velocity_lpf.filt(current_velocity)

        steering = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        velocity_error = linear_velocity - current_velocity

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_control.step(velocity_error, sample_time)

        brake = 0.0
        if abs(linear_velocity) < 1e-9 and current_velocity < 0.1:
            throttle = 0.0
            brake = 400
        elif throttle < 0.1 and velocity_error < 0:
            decel = max(velocity_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering

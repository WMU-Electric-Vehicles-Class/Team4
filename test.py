import numpy as np

MPH_TO_MPS = 0.44704
INCH_TO_M = 0.0254

vel_mph = 10  # mph
tire_radius_in = 15.02  # in

vel = 10 * MPH_TO_MPS

tire_radius = tire_radius_in * INCH_TO_M
print(tire_radius)

# v = ω * r
tire_angular_velocity = vel / tire_radius
print(tire_angular_velocity)

front_ratio = 9.205

# r = ωin / ωout
motor_angular_velocity = front_ratio * tire_angular_velocity

print(motor_angular_velocity)

rpm = motor_angular_velocity * 60 / (2 * np.pi)
print(rpm)

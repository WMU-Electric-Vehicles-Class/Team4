import numpy as np
import pandas as pd
import scipy.integrate


class e_tron:
    class Wheel:
        def __init__(self, parent):
            self.parent = parent

            INCH_TO_M = 0.0254

            tire_radius_in = 15.02  # inches  https://www.wheel-size.com/size/audi/e-tron/2020/  https://www.1010tires.com/Tools/Tire-Size-Calculator/255-55R19
            self.tire_radius = tire_radius_in * INCH_TO_M  # meters

        def step(self):
            self.angular_velocity = self.parent.velocity / self.tire_radius

    class Gearbox:
        def __init__(self, parent):
            self.parent = parent

            # https://electrichasgoneaudi.net/models/e-tron/drivetrain/motor/
            # https://static.nhtsa.gov/odi/tsbs/2019/MC-10155750-9999.pdf
            # Front gearbox torque ratio
            self.front_ratio = 9.205
            # Rear gearbox torque ratio
            self.rear_ratio = 9.083

        def step(self):
            wheel_angular_velocity = self.parent.wheel.angular_velocity

            self.front_motor_speed = wheel_angular_velocity * self.front_ratio
            self.rear_motor_speed = wheel_angular_velocity * self.rear_ratio

    class Motor:
        def __init__(self, parent):
            self.parent = parent

            self.p = 4  # Number of poles
            self.voltage = 360  # V

        def front_torque_Nm(self, rpm):
            # Piecewise front motor torque vs. speed curve

            x1, y1 = 0, 305
            x2, y2 = 4250, 305
            x3, y3 = 9500, 115
            x4, y4 = 13500, 65

            m1 = (y2 - y1) / (x2 - x1)
            m2 = (y3 - y2) / (x3 - x2)
            m3 = (y4 - y3) / (x4 - x3)

            if rpm == 0:
                return 0
            elif x1 <= rpm and rpm < x2:
                return m1 * (rpm - x1) + y1
            elif x2 <= rpm and rpm < x3:
                return m2 * (rpm - x2) + y2
            elif x3 <= rpm and rpm <= x4:
                return m3 * (rpm - x3) + y3
            else:
                raise Exception("Motor velocity out of bounds")

        def front_power_kW(self, rpm):
            # Piecewise front motor power vs. speed curve

            x1, y1 = 0, 0
            x2, y2 = 4250, 140
            x3, y3 = 8250, 140
            x4, y4 = 13500, 85

            m1 = (y2 - y1) / (x2 - x1)
            m2 = (y3 - y2) / (x3 - x2)
            m3 = (y4 - y3) / (x4 - x3)

            if x1 <= rpm and rpm < x2:
                return m1 * (rpm - x1) + y1
            elif x2 <= rpm and rpm < x3:
                return m2 * (rpm - x2) + y2
            elif x3 <= rpm and rpm <= x4:
                return m3 * (rpm - x3) + y3
            else:
                raise Exception("Motor velocity out of bounds")

        def rear_torque_Nm(self, rpm):
            # Piecewise rear motor torque vs. speed curve

            x1, y1 = 0, 360
            x2, y2 = 4800, 360
            x3, y3 = 8800, 165
            x4, y4 = 13500, 65

            m1 = (y2 - y1) / (x2 - x1)
            m2 = (y3 - y2) / (x3 - x2)
            m3 = (y4 - y3) / (x4 - x3)

            if rpm == 0:
                return 0
            elif x1 < rpm and rpm < x2:
                return m1 * (rpm - x1) + y1
            elif x2 <= rpm and rpm < x3:
                return m2 * (rpm - x2) + y2
            elif x3 <= rpm and rpm <= x4:
                return m3 * (rpm - x3) + y3
            else:
                raise Exception("Motor velocity out of bounds")

        def rear_power_kW(self, rpm):
            # Piecewise rear motor power vs. speed curve

            x1, y1 = 0, 0
            x2, y2 = 4250, 173
            x3, y3 = 8000, 173
            x4, y4 = 13500, 130

            m1 = (y2 - y1) / (x2 - x1)
            m2 = (y3 - y2) / (x3 - x2)
            m3 = (y4 - y3) / (x4 - x3)

            if x1 <= rpm and rpm < x2:
                return m1 * (rpm - x1) + y1
            elif x2 <= rpm and rpm < x3:
                return m2 * (rpm - x2) + y2
            elif x3 <= rpm and rpm <= x4:
                return m3 * (rpm - x3) + y3
            else:
                raise Exception("Motor velocity out of bounds")

        def step(self):
            self.speed_front = self.parent.gearbox.front_motor_speed
            self.speed_rear = self.parent.gearbox.rear_motor_speed

            # Calculate RPM
            self.nmr_front = self.front_angular_velocity * 60 / (2 * np.pi)
            self.nmr_rear = self.rear_angular_velocity * 60 / (2 * np.pi)

            # Calculate output torque
            self.Tm_front = self.front_torque_Nm(self.front_nmr)
            self.Tm_rear = self.rear_torque_Nm(self.rear_nmr)

            # Calculate output power
            self.Pout_front = self.front_power_kW(self.front_nmr)
            self.Pout_rear = self.rear_power_kW(self.rear_nmr)

            # Calculate supply frequencies
            # nmr = 120 * fs / p
            self.fs_front = self.p / 120 * self.front_nmr  # Front supply frequency, Hz
            self.fs_rear = self.p / 120 * self.rear_nmr

    class Battery:
        def __init__(self, parent, SOC_initial):
            self.parent = parent

            self.voltage = 396  # Nominal battery voltage, V

            self.max_energy_capacity_kWh = 95  # Maximum battery energy capacity
            self.min_energy_capacity_kWh = 8.5  # Minimum battery energy capacity  https://ev-database.org/car/1253/Audi-e-tron-55-quattro#charge-table

            self.SOC = SOC_initial
            self.min_SOC = self.min_energy_capacity_kWh / self.max_energy_capacity_kWh

        def is_depleted(self):
            return self.SOC <= self.min_SOC

        def step(self):
            pass

    def __init__(self, SOC_initial=1.00):
        self.wheel = self.Wheel(self)
        self.gearbox = self.Gearbox(self)
        self.motor = self.Motor(self)
        self.battery = self.Battery(self, SOC_initial)

    def simulate(self, cycle):
        self.cycle = cycle

        # Convert velocity to meters per second
        MPH_TO_MPS = 0.44704
        cycle["Velocity (m/s)"] = cycle["Velocity (mph)"] * MPH_TO_MPS

        for i in range(len(self.cycle)):
            self.time_s = self.cycle["Time (s)"].iloc[i]
            self.velocity = self.cycle["Velocity (m/s)"].iloc[i]

            # Vehicle dynamics calculations
            if i > 0:
                self.prev_time_s = self.cycle["Time (s)"].iloc[i - 1]
                self.prev_velocity = self.cycle["Velocity (m/s)"].iloc[i - 1]

                self.dt = self.time_s - self.prev_time_s
                self.dv = self.velocity - self.prev_velocity

                # Change in distance
                self.ds = scipy.integrate.trapezoid(
                    y=[self.prev_velocity, self.velocity],
                    dx=self.dt,
                )

                self.acceleration = self.dv / self.dt
            else:
                self.ds = 0
                self.acceleration = 0

            self.wheel.step()
            self.gearbox.step()
            self.motor.step()
            self.battery.step()

            self.log_data()

    def log_data(self):
        pass


"""a = []
b = []"""

my_fancy_ev = e_tron()

hwfet = pd.read_csv("hwycol.txt", sep="\t")

my_fancy_ev.simulate(hwfet)

"""from matplotlib import pyplot as plt

plt.plot(list(range(len(a))), a)
plt.plot(list(range(len(b))), b)
plt.show()"""

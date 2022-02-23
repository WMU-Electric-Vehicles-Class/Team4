import numpy as np
import pandas as pd
import scipy.integrate


class e_tron:
    class Chassis:
        def __init__(self, parent):
            self.parent = parent

            # Mass
            self.curb_weight = 2490  # Mass without cargo or passengers  https://www.evspecifications.com/en/model/bad6f3

            # Aerodynamics
            self.rho = 1.225  # Density of air at STP, kg/m^3
            self.Cd = 0.28  # Coefficient of drag  https://www.evspecifications.com/en/model/bad6f3
            self.A = 2.65  # Frontal area, m^2 https://www.carfolio.com/audi-e-tron-55-quattro-602580

            # Rolling resistance
            self.mu_rr = 0.015
            # Frr = µrr * m * g
            self.rr_force = self.mu_rr * self.curb_weight * 9.81

        def step(self):
            # F = m * a
            self.net_force = self.curb_weight * self.parent.acceleration
            # Fd = 1/2 * ρ * v^2 * Cd * A
            self.drag_force = (
                1 / 2 * self.rho * self.parent.velocity**2 * self.Cd * self.A
            )
            # Sum of horizontal forces: F = Fw - Fd - Frr
            # Ft = F + Fd + Frr
            all_wheel_force = self.net_force + self.drag_force + self.rr_force
            if all_wheel_force >= 0:
                self.traction_force = all_wheel_force
                self.braking_force = 0
            elif all_wheel_force < 0:
                self.traction_force = 0
                self.braking_force = all_wheel_force
            self.traction_force_per_wheel = self.traction_force / 4
            # This force is for every wheel (e-trons are AWD), positive is traction, negative is braking

    class Wheel:
        def __init__(self, parent):
            self.parent = parent

            INCH_TO_M = 0.0254

            tire_radius_in = 15.02  # inches  https://www.wheel-size.com/size/audi/e-tron/2020/  https://www.1010tires.com/Tools/Tire-Size-Calculator/255-55R19
            self.tire_radius = tire_radius_in * INCH_TO_M  # meters

        def step(self):
            self.force = self.parent.chassis.traction_force_per_wheel
            self.torque = self.force * self.tire_radius  # N*m
            self.angular_velocity = self.parent.velocity / self.tire_radius

    class Gearbox:
        def __init__(self, parent):
            self.parent = parent

            # https://electrichasgoneaudi.net/models/e-tron/drivetrain/motor/
            # Front gearbox ratio (APA250)
            self.front_ratio = 9.205
            # Rear gearbox ratio (AKA320)
            self.rear_ratio = 9.08

        def step(self):
            wheel_angular_velocity = self.parent.wheel.angular_velocity
            wheel_torque = self.parent.wheel.torque

            self.front_motor_speed = wheel_angular_velocity * self.front_ratio
            self.front_motor_torque = wheel_torque / self.front_ratio

            self.rear_motor_speed = wheel_angular_velocity * self.rear_ratio
            self.rear_motor_torque = wheel_torque / self.rear_ratio

    class Motor:
        def __init__(self, parent):
            self.parent = parent

            # Number of poles
            self.p = 4

            # Front motor (EASA)
            # Torque by angular velocity
            # https://static.nhtsa.gov/odi/tsbs/2019/MC-10155750-9999.pdf

            # Rear motor (EAWA)

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
            self.front_angular_velocity = self.parent.gearbox.front_motor_speed
            self.front_rpm = self.front_angular_velocity * 60 / (2 * np.pi)
            self.front_torque = self.parent.gearbox.front_motor_torque
            print(self.front_torque, self.front_torque_Nm(self.front_rpm))

            self.rear_angular_velocity = self.parent.gearbox.rear_motor_speed
            self.rear_torque = self.parent.gearbox.rear_motor_torque

    class Battery:
        def __init__(self, parent, SOC_initial):
            self.parent = parent

            self.max_energy_capacity_kWh = 95  # Maximum battery energy capacity
            self.min_energy_capacity_kWh = 8.5  # Minimum battery energy capacity  https://ev-database.org/car/1253/Audi-e-tron-55-quattro#charge-table

            self.SOC = SOC_initial
            self.min_SOC = self.min_energy_capacity_kWh / self.max_energy_capacity_kWh

        def is_depleted(self):
            return self.SOC <= self.min_SOC

        def step(self):
            pass

    def __init__(self, SOC_initial=1.00):
        self.chassis = self.Chassis(self)
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

            self.chassis.step()
            self.wheel.step()
            self.gearbox.step()
            self.motor.step()
            self.battery.step()

            self.log_data()

    def log_data(self):
        pass


my_fancy_ev = e_tron()

hwfet = pd.read_csv("hwycol.txt", sep="\t")

my_fancy_ev.simulate(hwfet)

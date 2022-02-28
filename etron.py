import numpy as np
import pandas as pd
import scipy.integrate


class e_tron:
    class Chassis:
        def __init__(self, parent):
            self.parent = parent

            g = 9.81

            self.mass = 2595  # kg
            self.weight = self.mass * g  # N

            # Aerodynamics
            self.rho = 1.225  # Density of air at STP, kg/m^3
            self.Cd = 0.25  # Coefficient of drag
            self.A = 2.65  # Frontal area, m^2 https://www.auk-kits.co.uk/assets/documents/original/8386-14AudietronAerodynamics.pdf

            # Rolling resistance, Frr = µrr * W
            self.mu_rr = 0.015
            self.F_rr = self.mu_rr * self.weight

        def step(self):
            road_angle = np.arctan(self.parent.grade)  # radians
            self.F_grade = self.weight * np.sin(road_angle)  # in negative x direction

            self.F_drag = (
                1 / 2 * self.rho * self.parent.velocity**2 * self.Cd * self.A
            )  # in negative x direction

            # Fx - Fdrag - Frr - Fgrade = m*a
            Fx = (
                self.mass * self.parent.acceleration
                + self.F_drag
                + self.F_rr
                + self.F_grade
            )

            if Fx >= 0:
                # Driving
                self.F_drive = Fx
                self.F_brake = 0
            elif Fx < 0:
                # Braking
                self.F_drive = 0
                self.F_brake = -Fx

    class Wheel:
        def __init__(self, parent):
            self.parent = parent

            INCH_TO_M = 0.0254
            tire_radius_in = (
                0.98 * 15.02
            )  # in  https://www.wheel-size.com/size/audi/e-tron/2020/  https://www.1010tires.com/Tools/Tire-Size-Calculator/255-55R19
            self.tire_radius = tire_radius_in * INCH_TO_M  # meters

        def step(self):
            # Driving torque TODO: add regenerative braking
            driving_torque = self.parent.chassis.F_drive * self.tire_radius
            self.torque_front = 0.40 * driving_torque
            self.torque_rear = 0.60 * driving_torque
            self.angular_velocity = self.parent.velocity / self.tire_radius

    class Gearbox:
        def __init__(self, parent):
            self.parent = parent

            # https://electrichasgoneaudi.net/models/e-tron/drivetrain/motor/
            # https://static.nhtsa.gov/odi/tsbs/2019/MC-10155750-9999.pdf
            # Front gearbox torque ratio
            self.ratio_front = 9.205
            # Rear gearbox torque ratio
            self.ratio_rear = 9.083

        def step(self):
            wheel_angular_velocity = self.parent.wheel.angular_velocity
            wheel_torque_front = self.parent.wheel.torque_front
            wheel_torque_rear = self.parent.wheel.torque_rear

            self.motor_torque_front = wheel_torque_front / self.ratio_front
            self.motor_speed_front = wheel_angular_velocity * self.ratio_front

            self.motor_torque_rear = wheel_torque_rear / self.ratio_rear
            self.motor_speed_rear = wheel_angular_velocity * self.ratio_rear

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
            # Front motor torque and speed
            self.torque_front = self.parent.gearbox.motor_torque_front
            self.speed_front = self.parent.gearbox.motor_speed_front
            self.rpm_front = self.speed_front * 60 / (2 * np.pi)

            # Rear motor torque and speed
            self.torque_rear = self.parent.gearbox.motor_torque_rear
            self.speed_rear = self.parent.gearbox.motor_speed_rear
            self.rpm_rear = self.speed_rear * 60 / (2 * np.pi)

            if not self.rpm_front == 0:
                # Calculate max output torque front
                self.torque_front_max = self.front_torque_Nm(self.rpm_front)
                self.control_signal_front = self.torque_front / self.torque_front_max

                # Calculate max output torque rear
                self.torque_rear_max = self.rear_torque_Nm(self.rpm_rear)
                self.control_signal_rear = self.torque_rear / self.torque_rear_max
            else:
                self.control_signal_front = 0
                self.control_signal_rear = 0

            # Calculate output power
            self.power_out_front = self.torque_front * self.speed_front
            self.power_out_rear = self.torque_rear * self.speed_rear
            a.append(self.power_out_front)
            b.append(self.power_out_rear)

            # Calculate input power
            eta_front = 1.00  # TODO: replace with efficiency map function
            eta_rear = 1.00  # TODO: replace with efficiency map function
            self.power_in_front = self.power_out_front / eta_front
            self.power_in_rear = self.power_out_rear / eta_rear

            # Calculate supply frequencies
            # nmr = 120 * fs / p
            self.fs_front = self.p / 120 * self.rpm_front  # Front supply frequency, Hz
            self.fs_rear = self.p / 120 * self.rpm_rear

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
            self.time = self.cycle["Time (s)"].iloc[i]
            self.velocity = self.cycle["Velocity (m/s)"].iloc[i]
            self.grade = 0  # TODO: replace

            # Vehicle dynamics calculations
            if i > 0:
                self.prev_time = self.cycle["Time (s)"].iloc[i - 1]
                self.prev_velocity = self.cycle["Velocity (m/s)"].iloc[i - 1]

                self.dt = self.time - self.prev_time
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


if __name__ == "__main__":
    a = []
    b = []

    my_fancy_ev = e_tron()

    hwfet = pd.read_csv("data/hwycol.txt", sep="\t")

    my_fancy_ev.simulate(hwfet)

    from matplotlib import pyplot as plt
    import seaborn as sns

    sns.set()
    a = np.array(a)
    b = np.array(b)

    plt.gcf().set_size_inches(3.5 * 3, 1.5 * 3)
    plt.plot(list(range(len(a))), a / 1000)
    plt.plot(list(range(len(b))), b / 1000)
    plt.xlabel("Time [sec]")
    plt.ylabel("Power [kW]")
    plt.legend(["Front motor power output", "Rear motor power output"])
    plt.tight_layout()
    plt.savefig("plot.png")

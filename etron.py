from operator import le
import numpy as np
import pandas as pd
import scipy.integrate
from scipy.interpolate import LinearNDInterpolator


class e_tron:
    class Chassis:
        def __init__(self, parent):
            self.parent = parent

            self.mass = 2595  # kg
            self.weight = self.mass * self.parent.g  # N

            # Aerodynamics
            self.rho = 1.225  # Density of air at STP, kg/m^3
            self.Cd = 0.25  # Coefficient of drag
            self.A = 2.65  # Frontal area, m^2 https://www.auk-kits.co.uk/assets/documents/original/8386-14AudietronAerodynamics.pdf

            # Rolling resistance
            self.mu_rr = 0.015

        def step(self):
            road_angle = np.arctan(self.parent.grade)  # radians
            self.F_grade = self.weight * np.sin(road_angle)  # in negative x direction

            # Rolling resistance force
            self.F_rr = self.mu_rr * self.weight if self.parent.velocity > 0 else 0

            # Drag force
            self.F_drag = (
                1 / 2 * self.rho * self.parent.velocity**2 * self.Cd * self.A
            )  # in negative x direction

            # Fx - Fdrag - Frr - Fgrade = m*a
            self.F_x = (
                self.mass * self.parent.acceleration
                + self.F_drag
                + self.F_rr
                + self.F_grade
            )

    class Wheel:
        def __init__(self, parent):
            self.parent = parent

            # Regenerative braking
            regen_max_decel_g = 0.3  # g
            self.regen_max_decel = regen_max_decel_g * self.parent.g  # m/s^2

            tire_radius_in = (
                0.98 * 15.02
            )  # in  https://www.wheel-size.com/size/audi/e-tron/2020/  https://www.1010tires.com/Tools/Tire-Size-Calculator/255-55R19
            self.tire_radius = tire_radius_in * self.parent.INCH_TO_M  # meters

        def step(self):
            F_x = self.parent.chassis.F_x

            self.total_torque = F_x * self.tire_radius
            self.torque_front = 0.40 * self.total_torque
            self.torque_rear = 0.60 * self.total_torque

            self.driving = self.total_torque >= 0

            self.regen_braking = (
                not self.driving
                and self.parent.acceleration < 0
                and self.parent.acceleration >= self.regen_max_decel
            )

            self.speed = self.parent.velocity / self.tire_radius

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
            wheel_speed = self.parent.wheel.speed
            wheel_torque_front = self.parent.wheel.torque_front
            wheel_torque_rear = self.parent.wheel.torque_rear

            self.motor_torque_front = wheel_torque_front / self.ratio_front
            self.motor_speed_front = wheel_speed * self.ratio_front

            self.motor_torque_rear = wheel_torque_rear / self.ratio_rear
            self.motor_speed_rear = wheel_speed * self.ratio_rear

    class Motor:
        def __init__(self, parent):
            self.parent = parent

            self.p = 4  # Number of poles
            self.voltage = 360  # V

            # Build front motor efficiency map
            front_eff_data = pd.read_excel(
                "data/Audi_e_Tron_Motors_ResultsForMaps.xlsx", "APA250-Maps"
            )
            x_front = front_eff_data["SPEED [rad/s]"]
            y_front = front_eff_data["TORQUE [Nm]"]
            z_front = front_eff_data["EFFY [%]"] / 100
            self.interp_eff_front = LinearNDInterpolator((x_front, y_front), z_front)

            # Build rear motor efficiency map
            rear_eff_data = pd.read_excel(
                "data/Audi_e_Tron_Motors_ResultsForMaps.xlsx", "AKA320-Maps"
            )
            x_rear = rear_eff_data["SPEED [rad/s]"]
            y_rear = rear_eff_data["TORQUE [Nm]"]
            z_rear = rear_eff_data["EFFY [%]"] / 100
            self.interp_eff_rear = LinearNDInterpolator((x_rear, y_rear), z_rear)

            self.prev_power_in_front = 0
            self.prev_power_in_rear = 0

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

            # Calculate output power
            self.power_out_front = self.torque_front * self.speed_front
            self.power_out_rear = self.torque_rear * self.speed_rear

            # Front motor efficiency
            self.efficiency_front = self.interp_eff_front(
                self.speed_front, abs(self.torque_front)
            )
            if np.isnan(self.efficiency_front):
                print(
                    f"WARNING: Not in front motor map: speed={self.speed_front:.2f} rad/s, torque={self.torque_front:.2f} Nm. Assuming 100% efficiency."
                )
                self.efficiency_front = 1.00
            # Rear motor efficiency
            self.efficiency_rear = self.interp_eff_rear(
                self.speed_rear, abs(self.torque_rear)
            )
            if np.isnan(self.efficiency_rear):
                print(
                    f"WARNING: Not in rear motor map: speed={self.speed_rear:.2f} rad/s, torque={self.torque_rear:.2f} Nm. Assuming 100% efficiency."
                )
                self.efficiency_rear = 1.00

            if self.parent.wheel.driving:
                # Driving: input power is greater than output power
                # Calculate front motor input power
                self.power_in_front = (
                    self.power_out_front / self.efficiency_front
                    if self.power_out_front >= 0
                    else 0
                )
                # Calculate rear motor input power
                self.power_in_rear = (
                    self.power_out_rear / self.efficiency_rear
                    if self.power_out_rear >= 0
                    else 0
                )
            else:
                # Braking: output power is greater than input power
                # Calculate front motor input power
                self.power_in_front = self.power_out_front * self.efficiency_front
                # Calculate rear motor input power
                self.power_in_rear = self.power_out_rear * self.efficiency_rear

            # Calculate supply frequencies
            # nmr = 120 * fs / p
            self.fs_front = self.p / 120 * self.rpm_front  # Front supply frequency, Hz
            self.fs_rear = self.p / 120 * self.rpm_rear

            # Calculate energy consumption over time step
            self.energy_in_front = scipy.integrate.trapezoid(
                y=[self.power_in_front, self.prev_power_in_front],
                dx=self.parent.dt,
            )
            self.energy_in_rear = scipy.integrate.trapezoid(
                y=[self.power_in_rear, self.prev_power_in_rear],
                dx=self.parent.dt,
            )
            self.energy_consumption = self.energy_in_front + self.energy_in_rear

            self.prev_power_in_rear = self.power_in_rear

    class Battery:
        def __init__(self, parent, soc_initial):
            self.parent = parent

            self.voltage = 396  # Nominal battery voltage, V

            # https://ev-database.org/car/1253/Audi-e-tron-55-quattro#charge-table
            max_energy_capacity_kWh = 95
            useable_energy_capacity_kWh = 86.5
            min_energy_capacity_kWh = (
                max_energy_capacity_kWh - useable_energy_capacity_kWh
            )

            self.max_energy_capacity = (
                max_energy_capacity_kWh * self.parent.KWH_TO_J
            )  # J
            self.min_energy_capacity = (
                min_energy_capacity_kWh * self.parent.KWH_TO_J
            )  # J

            self.soc = soc_initial
            self.remaining_energy = self.soc * self.max_energy_capacity  # J

        def step(self):
            # TODO: include auxiliary loads
            self.energy_consumption = self.parent.motor.energy_consumption
            # print(energy_consumption, self.remaining_energy)

            self.remaining_energy -= self.energy_consumption
            self.soc = self.remaining_energy / self.max_energy_capacity

            self.depleted = self.remaining_energy <= self.min_energy_capacity

    class Logger:
        def __init__(self, parent):
            self.parent = parent
            length = len(self.parent.cycle)
            # Time
            self.time_s = np.zeros(length)
            # Velocity
            self.velocity_mps = np.zeros(length)
            self.velocity_mph = np.zeros(length)
            # Acceleration
            self.acceleration_mps2 = np.zeros(length)
            self.acceleration_g = np.zeros(length)
            # Distance
            self.distance = np.zeros(length)
            self.distance_mi = np.zeros(length)
            # Grade
            self.grade = np.zeros(length)
            # Chassis
            self.F_grade = np.zeros(length)
            self.F_rr = np.zeros(length)
            self.F_drag = np.zeros(length)
            self.F_x = np.zeros(length)
            # Wheel
            self.driving = np.zeros(length)
            self.wheel_torque_front = np.zeros(length)
            self.wheel_torque_rear = np.zeros(length)
            self.regen_braking = np.zeros(length)
            self.wheel_speed = np.zeros(length)
            # Motor
            self.motor_torque_front = np.zeros(length)
            self.motor_torque_rear = np.zeros(length)
            self.motor_speed_front = np.zeros(length)
            self.motor_speed_rear = np.zeros(length)
            self.motor_p_out_front = np.zeros(length)
            self.motor_p_out_rear = np.zeros(length)
            self.motor_effy_front = np.zeros(length)
            self.motor_effy_rear = np.zeros(length)
            self.motor_p_in_front = np.zeros(length)
            self.motor_p_in_rear = np.zeros(length)
            self.motor_energy_cons_front = np.zeros(length)
            self.motor_energy_cons_rear = np.zeros(length)
            self.motor_energy_cons = np.zeros(length)
            # Battery
            self.battery_energy = np.zeros(length)
            self.soc = np.zeros(length)
            self.battery_depleted = np.zeros(length)

        def log_data(self):
            i = self.parent.i
            # Time
            self.time_s[i] = self.parent.time
            # Velocity
            self.velocity_mps[i] = self.parent.velocity
            self.velocity_mph[i] = self.velocity_mph[i] / self.parent.MPH_TO_MPS
            # Acceleration
            self.acceleration_mps2[i] = self.parent.acceleration
            self.acceleration_g[i] = self.acceleration_mps2[i] / self.parent.g
            # Distance
            self.distance[i] = self.distance[i - 1] + self.parent.ds
            self.distance_mi[i] = self.distance[i] * self.parent.M_TO_MI
            # Grade
            self.grade[i] = self.parent.grade
            # Chassis
            self.F_grade[i] = self.parent.chassis.F_grade
            self.F_rr[i] = self.parent.chassis.F_rr
            self.F_drag[i] = self.parent.chassis.F_drag
            self.F_grade[i] = self.parent.chassis.F_x
            # Wheel
            self.driving[i] = self.parent.wheel.driving
            self.wheel_torque_front[i] = self.parent.wheel.torque_front
            self.wheel_torque_rear[i] = self.parent.wheel.torque_rear
            self.regen_braking[i] = self.parent.wheel.regen_braking
            self.wheel_speed[i] = self.parent.wheel.speed
            # Motor
            self.motor_torque_front[i] = self.parent.motor.torque_front
            self.motor_torque_rear[i] = self.parent.motor.torque_rear
            self.motor_speed_front[i] = self.parent.motor.speed_front
            self.motor_speed_rear[i] = self.parent.motor.speed_rear
            self.motor_p_out_front[i] = self.parent.motor.power_out_front
            self.motor_p_out_rear[i] = self.parent.motor.power_out_rear
            self.motor_effy_front[i] = self.parent.motor.efficiency_front
            self.motor_effy_rear[i] = self.parent.motor.efficiency_rear
            self.motor_p_in_front[i] = self.parent.motor.power_in_front
            self.motor_p_in_rear[i] = self.parent.motor.power_in_rear
            self.motor_energy_cons_front[i] = self.parent.motor.energy_in_front
            self.motor_energy_cons_rear[i] = self.parent.motor.energy_in_rear
            self.motor_energy_cons[i] = self.parent.motor.energy_consumption
            # Battery
            self.battery_energy[i] = self.parent.battery.remaining_energy
            self.soc[i] = self.parent.battery.soc
            self.battery_depleted[i] = self.parent.battery.depleted

    def __init__(self, soc_initial=0.90):
        self.g = 9.81
        # Conversion factors
        self.MPH_TO_MPS = 0.44704
        self.INCH_TO_M = 0.0254
        self.KWH_TO_J = 3.6e6
        self.M_TO_MI = 0.000621371

        self.chassis = self.Chassis(self)
        self.wheel = self.Wheel(self)
        self.gearbox = self.Gearbox(self)
        self.motor = self.Motor(self)
        self.battery = self.Battery(self, soc_initial)

    def simulate(self, cycle):
        self.cycle = cycle
        # Initialize data logger
        self.log = self.Logger(self)

        # Convert velocity to meters per second
        cycle["Velocity (m/s)"] = cycle["Velocity (mph)"] * self.MPH_TO_MPS

        for i in range(len(self.cycle)):
            self.i = i
            self.time = self.cycle["Time (s)"].iloc[i]
            self.velocity = self.cycle["Velocity (m/s)"].iloc[i]
            self.grade = 0  # TODO: replace

            # Vehicle dynamics calculations
            if i > 0:
                self.prev_velocity = self.cycle["Velocity (m/s)"].iloc[i - 1]
                self.prev_time = self.cycle["Time (s)"].iloc[i - 1]

                dv = self.velocity - self.prev_velocity
                self.dt = self.time - self.prev_time

                # Change in distance
                self.ds = scipy.integrate.trapezoid(
                    y=[self.prev_velocity, self.velocity],
                    dx=self.dt,
                )

                self.acceleration = dv / self.dt
            else:
                self.dt
                self.ds = 0
                self.acceleration = 0

            self.chassis.step()
            self.wheel.step()
            self.gearbox.step()
            self.motor.step()
            self.battery.step()

            self.log.log_data()


if __name__ == "__main__":
    my_fancy_ev = e_tron()

    hwfet = pd.read_csv("data/hwycol.txt", sep="\t")

    my_fancy_ev.simulate(hwfet)

    from matplotlib import pyplot as plt
    import seaborn as sns

    sns.set()

    x = my_fancy_ev.log.time_s
    y = my_fancy_ev.log.soc

    plt.rcParams["font.family"] = "Times New Roman"
    fig = plt.figure(figsize=(10, 4))
    ax1 = fig.add_subplot(111)

    ax1.plot(x, y * 100, color="C0")
    ax1.set_title("Audi e-tron: SOC vs. Time", fontsize="large", fontweight="bold")
    ax1.set_xlabel("Time [sec]", fontsize="large")
    ax1.set_ylabel("SOC [%]", fontsize="large")

    fig.tight_layout()
    plt.savefig("plots/SOC vs time regen new.png")

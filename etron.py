from operator import le
import numpy as np
import pandas as pd
import scipy.integrate
from scipy.interpolate import LinearNDInterpolator


class e_tron:
    class Chassis:
        def __init__(self, parent):
            self.parent = parent

            self.mass = 2595 + 1.573 + 3 + 20  # kg
            self.weight = self.mass * self.parent.g  # N

            # Aerodynamics
            self.rho = 1.225  # Density of air at STP, kg/m^3
            self.Cd = 0.25  # Coefficient of drag
            self.A = 2.65  # Frontal area, m^2 https://www.auk-kits.co.uk/assets/documents/original/8386-14AudietronAerodynamics.pdf

            # Rolling resistance
            self.mu_rr = 0.010

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

            self.parent.driving = self.F_x >= 0
            self.parent.braking = not self.parent.driving

    class Wheel:
        def __init__(self, parent):
            self.parent = parent

            # Regenerative braking
            regen_max_decel_g = -0.3  # g
            self.regen_max_decel = regen_max_decel_g * self.parent.g  # m/s^2
            self.regen_efficiency = 0.70

            tire_radius_in = (
                0.98 * 15.02
            )  # in  https://www.wheel-size.com/size/audi/e-tron/2020/  https://www.1010tires.com/Tools/Tire-Size-Calculator/255-55R19
            self.tire_radius = tire_radius_in * self.parent.INCH_TO_M  # meters

        def step(self):
            F_x = self.parent.chassis.F_x

            self.total_torque = F_x * self.tire_radius

            # Regenerative braking
            if self.parent.driving:
                self.regen_braking = False
            elif self.parent.braking and (
                self.parent.acceleration >= self.regen_max_decel
            ):
                self.total_torque *= self.regen_efficiency
                self.regen_braking = True
            else:
                self.total_torque = 0
                self.regen_braking = False

            self.torque_front = 0.40 * self.total_torque
            self.torque_rear = 0.60 * self.total_torque

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

            self.efficiency_front = 1.00 - 0.02 - 0.02 - 0.01 - 0.02
            self.efficiency_rear = 1.00 - 0.02 - 0.01 - 0.01 - 0.02

        def step(self):
            wheel_speed = self.parent.wheel.speed
            wheel_torque_front = self.parent.wheel.torque_front
            wheel_torque_rear = self.parent.wheel.torque_rear

            self.motor_torque_front = wheel_torque_front / self.ratio_front
            self.motor_speed_front = wheel_speed * self.ratio_front

            self.motor_torque_rear = wheel_torque_rear / self.ratio_rear
            self.motor_speed_rear = wheel_speed * self.ratio_rear

            if self.parent.driving:
                self.motor_torque_front /= self.efficiency_front
                self.motor_torque_rear /= self.efficiency_rear
            elif self.parent.braking:
                self.motor_torque_front *= self.efficiency_front
                self.motor_torque_rear *= self.efficiency_rear

    class Motor:
        def __init__(self, parent):
            self.parent = parent

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

            # Efficiency to use if torque+speed not in motor map
            self.fallback_efficiency = 0.95

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
                if self.parent.verbose:
                    print(
                        f"WARNING: Not in front motor map: speed={self.speed_front:.2f} rad/s, torque={self.torque_front:.2f} Nm. Assuming {self.fallback_efficiency*100}% efficiency."
                    )
                self.efficiency_front = self.fallback_efficiency
            # Rear motor efficiency
            self.efficiency_rear = self.interp_eff_rear(
                self.speed_rear, abs(self.torque_rear)
            )
            if np.isnan(self.efficiency_rear):
                if self.parent.verbose:
                    print(
                        f"WARNING: Not in rear motor map: speed={self.speed_rear:.2f} rad/s, torque={self.torque_rear:.2f} Nm. Assuming {self.fallback_efficiency*100}% efficiency."
                    )
                self.efficiency_rear = self.fallback_efficiency

            # Calculate motor input power
            if self.parent.driving:
                self.power_in_front = self.power_out_front / self.efficiency_front
                self.power_in_rear = self.power_out_rear / self.efficiency_rear
            elif self.parent.braking:
                self.power_in_front = self.power_out_front * self.efficiency_front
                self.power_in_rear = self.power_out_rear * self.efficiency_rear
            self.input_power = self.power_in_front + self.power_in_rear

    class Photovoltaics:
        def __init__(self, parent):
            self.parent = parent

            self.area = 3.7026  # m^2
            self.efficiency = 0.243

        def step(self):
            self.power_generated = self.parent.solar_irradiance * self.area  # W
            self.power_generated *= self.efficiency

    class PowerElectronics:
        def __init__(self, parent):
            self.parent = parent

            self.motor_elec_efficiency = 0.98
            self.pv_elec_efficiency = 0.98

            self.prev_power_cons = 0

        def step(self):
            if self.parent.driving:
                self.power_cons = (
                    self.parent.motor.input_power / self.motor_elec_efficiency
                )
            elif self.parent.braking:
                self.power_cons = (
                    self.parent.motor.input_power * self.motor_elec_efficiency
                )
            self.power_gain = (
                self.parent.photovoltaics.power_generated * self.pv_elec_efficiency
            )

            self.power_cons = self.power_cons - self.power_gain

            self.energy_cons = scipy.integrate.trapezoid(
                y=[self.power_cons, self.prev_power_cons],
                dx=self.parent.dt,
            )
            self.prev_power_cons = self.power_cons

    class Battery:
        def __init__(self, parent, soc_initial):
            self.parent = parent

            # https://ev-database.org/car/1253/Audi-e-tron-55-quattro#charge-table
            max_energy_capacity_kWh = 95.04
            usable_energy_capacity_kWh = 86.5
            min_energy_capacity_kWh = (
                max_energy_capacity_kWh - usable_energy_capacity_kWh
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
            self.energy_cons = self.parent.power_electronics.energy_cons

            self.remaining_energy -= self.energy_cons
            self.soc = self.remaining_energy / self.max_energy_capacity

            self.depleted = self.remaining_energy <= self.min_energy_capacity

    class Logger(dict):
        def __init__(self, parent):
            self.__dict__ = self
            self.parent = parent
            length = len(self.parent.cycle)

            # Time
            self["Time (s)"] = np.full(length, np.nan)
            # Velocity
            self["Velocity (m/s)"] = np.full(length, np.nan)
            self["Velocity (mph)"] = np.full(length, np.nan)
            # Acceleration
            self["Acceleration (m/s^2)"] = np.full(length, np.nan)
            self["Acceleration (g)"] = np.full(length, np.nan)
            # Distance
            self["Distance (m)"] = np.full(length, np.nan)
            self["Distance (mi)"] = np.full(length, np.nan)
            # Grade
            self["Grade"] = np.full(length, np.nan)
            # Chassis
            self["Fgrade"] = np.full(length, np.nan)
            self["Frr"] = np.full(length, np.nan)
            self["Fdrag"] = np.full(length, np.nan)
            self["Fx"] = np.full(length, np.nan)
            # Wheel
            self["Driving"] = np.full(length, np.nan)
            self["Braking"] = np.full(length, np.nan)
            self["Front wheel torque"] = np.full(length, np.nan)
            self["Rear wheel torque"] = np.full(length, np.nan)
            self["Regenerative braking"] = np.full(length, np.nan)
            self["Wheel speed (rad/s)"] = np.full(length, np.nan)
            # Motor
            self["Front motor torque (Nm)"] = np.full(length, np.nan)
            self["Rear motor torque (Nm)"] = np.full(length, np.nan)
            self["Front motor speed (rad/s)"] = np.full(length, np.nan)
            self["Rear motor speed (rad/s)"] = np.full(length, np.nan)
            self["Front motor power (W)"] = np.full(length, np.nan)
            self["Rear motor power (W)"] = np.full(length, np.nan)
            self["Front motor efficiency"] = np.full(length, np.nan)
            self["Rear motor efficiency"] = np.full(length, np.nan)
            self["Front motor input power (W)"] = np.full(length, np.nan)
            self["Rear motor input power (W)"] = np.full(length, np.nan)
            self["Motor input power (W)"] = np.full(length, np.nan)
            # Photovoltaics
            self["PV power generated (W)"] = np.full(length, np.nan)
            # Power Electronics
            self["Power consumed (W)"] = np.full(length, np.nan)
            self["Power generated (W)"] = np.full(length, np.nan)
            self["Energy consumed (J)"] = np.full(length, np.nan)
            # Battery
            self["Battery energy (J)"] = np.full(length, np.nan)
            self["SOC"] = np.full(length, np.nan)
            self["Battery depleted"] = np.full(length, np.nan)

        def log_data(self):
            i = self.parent.i
            # Time
            self["Time (s)"][i] = self.parent.time
            # Velocity
            self["Velocity (m/s)"][i] = self.parent.velocity
            self["Velocity (mph)"][i] = (
                self["Velocity (m/s)"][i] / self.parent.MPH_TO_MPS
            )
            # Acceleration
            self["Acceleration (m/s^2)"][i] = self.parent.acceleration
            self["Acceleration (g)"][i] = (
                self["Acceleration (m/s^2)"][i] / self.parent.g
            )
            # Distance
            if i == 0:
                self["Distance (m)"][i] = self.parent.ds
            else:
                self["Distance (m)"][i] = self["Distance (m)"][i - 1] + self.parent.ds
            self["Distance (mi)"][i] = self["Distance (m)"][i] * self.parent.M_TO_MI
            # print(self["Distance (mi)"][i])
            # Grade
            self["Grade"][i] = self.parent.grade
            # Chassis
            self["Fgrade"][i] = self.parent.chassis.F_grade
            self["Frr"][i] = self.parent.chassis.F_rr
            self["Fdrag"][i] = self.parent.chassis.F_drag
            self["Fx"][i] = self.parent.chassis.F_x
            # Wheel
            self["Driving"][i] = self.parent.driving
            self["Braking"][i] = self.parent.braking
            self["Front wheel torque"][i] = self.parent.wheel.torque_front
            self["Rear wheel torque"][i] = self.parent.wheel.torque_rear
            self["Regenerative braking"][i] = self.parent.wheel.regen_braking
            self["Wheel speed (rad/s)"][i] = self.parent.wheel.speed
            # Motor
            self["Front motor torque (Nm)"][i] = self.parent.motor.torque_front
            self["Rear motor torque (Nm)"][i] = self.parent.motor.torque_rear
            self["Front motor speed (rad/s)"][i] = self.parent.motor.speed_front
            self["Rear motor speed (rad/s)"][i] = self.parent.motor.speed_rear
            self["Front motor power (W)"][i] = self.parent.motor.power_out_front
            self["Rear motor power (W)"][i] = self.parent.motor.power_out_rear
            self["Front motor efficiency"][i] = self.parent.motor.efficiency_front
            self["Rear motor efficiency"][i] = self.parent.motor.efficiency_rear
            self["Front motor input power (W)"][i] = self.parent.motor.power_in_front
            self["Rear motor input power (W)"][i] = self.parent.motor.power_in_rear
            self["Motor input power (W)"][i] = self.parent.motor.input_power
            # Photovoltaics
            self["PV power generated (W)"][
                i
            ] = self.parent.photovoltaics.power_generated
            # Power Electronics
            self["Power consumed (W)"][i] = self.parent.power_electronics.power_cons
            self["Power generated (W)"][i] = self.parent.power_electronics.power_gain
            self["Energy consumed (J)"][i] = self.parent.power_electronics.energy_cons
            # Battery
            self["Battery energy (J)"][i] = self.parent.battery.remaining_energy
            self["SOC"][i] = self.parent.battery.soc
            self["Battery depleted"][i] = self.parent.battery.depleted

    def __init__(self, initial_soc=1.00, verbose=False):
        self.verbose = verbose

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
        self.photovoltaics = self.Photovoltaics(self)
        self.power_electronics = self.PowerElectronics(self)
        self.battery = self.Battery(self, initial_soc)

        self.grade = 0
        self.solar_irradiance = 0

    def simulate(self, cycle):
        self.cycle = cycle
        # Initialize data logger
        self.log = self.Logger(self)

        # Convert velocity to meters per second
        self.cycle["Velocity (m/s)"] = self.cycle["Velocity (mph)"] * self.MPH_TO_MPS

        for i in range(len(self.cycle)):
            self.i = i
            self.time = self.cycle["Time (s)"].iloc[i]
            self.velocity = self.cycle["Velocity (m/s)"].iloc[i]

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
                self.dt = 1
                self.ds = 0
                self.acceleration = 0

            self.chassis.step()
            self.wheel.step()
            self.gearbox.step()
            self.motor.step()
            self.photovoltaics.step()
            self.power_electronics.step()
            self.battery.step()

            self.log.log_data()

            if self.battery.depleted:
                break

        for key in self.log:
            if type(self.log[key]) is np.ndarray:
                self.log[key] = self.log[key][~np.isnan(self.log[key])]


if __name__ == "__main__":
    cycle = pd.read_csv("data/wltp-3.csv", sep="\t")

    dist_mi = 0
    soc = 1.00

    print("Simulating...")
    while True:
        ev = e_tron(initial_soc=soc)
        ev.simulate(cycle)
        soc = ev.log["SOC"][-1]
        dist_mi += ev.log["Distance (mi)"][-1]
        if ev.battery.depleted:
            break
    print(f"Range: {dist_mi:.06f} mi")

"""Simulate Peltier cooler with non-linear efficiency and PID control."""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize  # type: ignore


class PeltierSimulator:
    """Peltier cooler simulator with non-linear efficiency."""

    def __init__(
        self,
        cooling_rate,
        heating_rate,
        peltier_heating_rate,
        target_temp,
        initial_temp,
    ):
        self.cooling_rate = (
            cooling_rate  # degrees Celsius per minute at 100% cooling power
        )
        self.heating_rate = (
            heating_rate  # degrees Celsius per minute when Peltier is off
        )
        self.peltier_heating_rate = (
            peltier_heating_rate  # Heating rate due to Peltier inefficiency
        )
        self.target_temp = (
            target_temp  # Target temperature to maintain inside the fridge
        )
        self.current_temp = initial_temp  # Current temperature

    def update(self, dt, cooling_power_percent):
        """
        Simulate one time step.
        cooling_power_percent: Percentage of cooling (0 to 100%), cannot drive in reverse.
        dt: Time step in minutes.
        """
        cooling_power_fraction = cooling_power_percent / 100.0  # Convert to fraction

        # Cooling effect (desired cooling)
        cooling_effect = -cooling_power_fraction * self.cooling_rate * dt

        # Heating effect due to Peltier inefficiency (non-linear term)
        peltier_heating_effect = (
            (cooling_power_fraction**2) * self.peltier_heating_rate * dt
        )

        # Environmental heating effect
        heating_effect = self.heating_rate * (1 - cooling_power_fraction) * dt

        # Update current temperature
        self.current_temp += cooling_effect + peltier_heating_effect + heating_effect

    def get_temperature(self):
        """Return the current temperature."""
        return self.current_temp


class PIDController:
    """PID controller for temperature control."""

    def __init__(self, Kp, Ki, Kd, mode="REVERSE"):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = None
        self.mode = mode  # 'DIRECT' or 'REVERSE'

    def update(self, setpoint, measured_value, dt):
        """
        Compute the PID control action.
        setpoint: The desired target value.
        measured_value: The current measured value.
        dt: Time step.
        """
        error = (
            setpoint - measured_value
            if self.mode == "DIRECT"
            else measured_value - setpoint
        )
        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = 0
        if self.previous_error is not None and dt != 0:
            derivative = (error - self.previous_error) / dt
        D = self.Kd * derivative

        # Update previous error for the next step
        self.previous_error = error

        # Compute the control action (cooling power between 0 and 100)
        output = P + I + D
        # Clamp between 0 and 100
        output = max(0.0, min(100.0, output))
        return output


def apply_peltier_clamping(cooling_power_percent, peltier_two_sided_clamp=5.0):
    """Apply the peltier clamping logic."""
    if cooling_power_percent < peltier_two_sided_clamp:
        # If duty cycle is less than threshold, set output LOW
        cooling_power_percent = 0.0
    elif cooling_power_percent > (100.0 - peltier_two_sided_clamp):
        # If duty cycle is greater than (100 - threshold), set output HIGH
        cooling_power_percent = 100.0
    # Else, cooling_power_percent remains as is
    return cooling_power_percent


def simulate_pid_control(
    Kp,
    Ki,
    Kd,
    cooling_rate,
    heating_rate,
    peltier_heating_rate,
    target_temp,
    initial_temp,
    simulation_time,
    dt,
):
    """Simulate the peltier system with PID control."""
    simulator = PeltierSimulator(
        cooling_rate, heating_rate, peltier_heating_rate, target_temp, initial_temp
    )
    pid_controller = PIDController(Kp, Ki, Kd, mode="REVERSE")

    time_points = np.arange(0, simulation_time, dt)
    temperature_history = []
    cooling_power_history = []

    for t in time_points:
        current_temp = simulator.get_temperature()
        cooling_power_percent = pid_controller.update(target_temp, current_temp, dt)
        # Apply the peltier clamping logic
        cooling_power_percent = apply_peltier_clamping(cooling_power_percent)
        simulator.update(dt, cooling_power_percent)

        temperature_history.append(current_temp)
        cooling_power_history.append(cooling_power_percent)

    return time_points, temperature_history, cooling_power_history


def objective_function(
    params,
    cooling_rate,
    heating_rate,
    peltier_heating_rate,
    target_temp,
    initial_temps,
    simulation_time,
    dt,
):
    """Objective function to minimize the squared error."""
    Kp, Ki, Kd = params
    total_squared_error = 0

    for initial_temp in initial_temps:
        _, temperature_history, _ = simulate_pid_control(
            Kp,
            Ki,
            Kd,
            cooling_rate,
            heating_rate,
            peltier_heating_rate,
            target_temp,
            initial_temp,
            simulation_time,
            dt,
        )
        error = np.array(temperature_history) - target_temp
        total_squared_error += np.sum(error**2)

    return total_squared_error


def plot_simulation_results(
    initial_temps,
    Kp_opt,
    Ki_opt,
    Kd_opt,
    cooling_rate,
    heating_rate,
    peltier_heating_rate,
    target_temp,
    simulation_time,
    dt,
):
    """Plot the simulation results for different initial temperatures."""
    fig, axs = plt.subplots(len(initial_temps), 2, figsize=(12, len(initial_temps) * 4))

    for i, initial_temp in enumerate(initial_temps):
        time_points, temperature_history, cooling_power_history = simulate_pid_control(
            Kp_opt,
            Ki_opt,
            Kd_opt,
            cooling_rate,
            heating_rate,
            peltier_heating_rate,
            target_temp,
            initial_temp,
            simulation_time,
            dt,
        )

        axs[i, 0].plot(
            time_points, temperature_history, label=f"Initial Temp: {initial_temp}°C"
        )
        axs[i, 0].axhline(target_temp, color="r", linestyle="--", label="Target Temp")
        axs[i, 0].set_xlabel("Time (minutes)")
        axs[i, 0].set_ylabel("Temperature (°C)")
        axs[i, 0].legend()
        axs[i, 0].set_title("Temperature vs. Time")

        axs[i, 1].plot(
            time_points,
            cooling_power_history,
            label=f"Initial Temp: {initial_temp}°C",
        )
        axs[i, 1].set_xlabel("Time (minutes)")
        axs[i, 1].set_ylabel("Cooling Power (%)")
        axs[i, 1].legend()
        axs[i, 1].set_title("Cooling Power vs. Time")

    plt.tight_layout()
    plt.show()


# Example usage
if __name__ == "__main__":
    # Simulation parameters
    cooling_rate = 0.06678571428  # degrees Celsius per minute at 100% power
    heating_rate = 0.03696488294  # degrees Celsius per minute when the Peltier is off
    peltier_heating_rate = 0.03  # Heating rate due to Peltier inefficiency at full power (adjust as needed)
    target_temp = 18  # Target temperature inside the fridge (in degrees Celsius)
    initial_temps = [20, 18.2, 17.8, 16, 18.9]  # Multiple initial temperatures to test
    simulation_time = 240  # Total time to simulate in minutes
    dt = 0.1  # Time step for simulation in minutes

    # Initial guesses for Kp, Ki, Kd (based on your Arduino code)
    initial_guess = [191.82, 2.0, 2.0]

    # Find the optimal Kp, Ki, Kd values using optimization
    result = minimize(
        objective_function,
        initial_guess,
        args=(
            cooling_rate,
            heating_rate,
            peltier_heating_rate,
            target_temp,
            initial_temps,
            simulation_time,
            dt,
        ),
        bounds=[(0, 1e6), (0, 1e4), (0, 1e4)],
        method="L-BFGS-B",
    )

    Kp_opt, Ki_opt, Kd_opt = result.x
    print(f"Optimal Kp: {Kp_opt}, Ki: {Ki_opt}, Kd: {Kd_opt}")

    # Plot simulation results for all initial temperatures
    plot_simulation_results(
        initial_temps,
        Kp_opt,
        Ki_opt,
        Kd_opt,
        cooling_rate,
        heating_rate,
        peltier_heating_rate,
        target_temp,
        simulation_time,
        dt,
    )

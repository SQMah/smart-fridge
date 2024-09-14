import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

COOLING_ALPHA = 0.05


class PeltierSimulator:
    def __init__(self, cooling_rate, heating_rate, target_temp, initial_temp):
        self.cooling_rate = (
            cooling_rate  # degrees Celsius per minute at 100% cooling power
        )
        self.heating_rate = (
            heating_rate  # degrees Celsius per minute when Peltier is off
        )
        self.target_temp = (
            target_temp  # Target temperature to maintain inside the fridge
        )
        self.current_temp = initial_temp  # Current temperature

    def update(self, dt, cooling_power):
        """
        Simulate one time step.
        cooling_power: Percentage of cooling (0 to 1.0), cannot drive in reverse.
        dt: Time step in minutes.
        """
        # Adjust cooling_power based on current temperature
        cooling_power = cooling_power - COOLING_ALPHA * (21 - self.current_temp - 3)
        # Clamp cooling_power between 0 and 1
        cooling_power = max(0.0, min(1.0, cooling_power))
        cooling_effect = -cooling_power * self.cooling_rate * dt
        heating_effect = self.heating_rate * (1 - cooling_power) * dt
        self.current_temp += cooling_effect + heating_effect

    def get_temperature(self):
        return self.current_temp


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = None

    def update(self, error, dt):
        """
        Compute the PID control action.
        error: The difference between the current temperature and the target temperature.
        dt: Time step.
        """
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

        # Compute the control action (cooling power between 0 and 1)
        output = P + I + D
        # Clamp between 0 and 1
        return max(0.0, min(1.0, output))


def simulate_pid_control(
    Kp,
    Ki,
    Kd,
    cooling_rate,
    heating_rate,
    target_temp,
    initial_temp,
    simulation_time,
    dt,
):
    simulator = PeltierSimulator(cooling_rate, heating_rate, target_temp, initial_temp)
    pid_controller = PIDController(Kp, Ki, Kd)

    time_points = np.arange(0, simulation_time, dt)
    temperature_history = []
    cooling_power_history = []

    for t in time_points:
        current_temp = simulator.get_temperature()
        error = current_temp - target_temp
        cooling_power = pid_controller.update(error, dt)
        simulator.update(dt, cooling_power)

        temperature_history.append(current_temp)
        cooling_power_history.append(cooling_power)

    return time_points, temperature_history, cooling_power_history


def objective_function(
    params, cooling_rate, heating_rate, target_temp, initial_temps, simulation_time, dt
):
    Kp, Ki, Kd = params
    total_squared_error = 0

    for initial_temp in initial_temps:
        _, temperature_history, _ = simulate_pid_control(
            Kp,
            Ki,
            Kd,
            cooling_rate,
            heating_rate,
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
    target_temp,
    simulation_time,
    dt,
):
    fig, axs = plt.subplots(len(initial_temps), 2, figsize=(12, len(initial_temps) * 4))

    for i, initial_temp in enumerate(initial_temps):
        time_points, temperature_history, cooling_power_history = simulate_pid_control(
            Kp_opt,
            Ki_opt,
            Kd_opt,
            cooling_rate,
            heating_rate,
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
        axs[i, 1].set_ylabel("Cooling Power (0-1)")
        axs[i, 1].legend()
        axs[i, 1].set_title("Cooling Power vs. Time")

    plt.tight_layout()
    plt.show()


# Example usage
if __name__ == "__main__":
    # Simulation parameters
    cooling_rate = 0.06678571428  # degrees Celsius per minute at 100% power
    heating_rate = 0.03696488294  # degrees Celsius per minute when the Peltier is off
    target_temp = 18  # Target temperature inside the fridge (in degrees Celsius)
    initial_temps = [20, 18.2, 17.8, 16, 18.9]  # Multiple initial temperatures to test
    simulation_time = 240  # Total time to simulate in minutes
    dt = 0.1  # Time step for simulation in minutes

    # Initial guesses for Kp, Ki, Kd
    initial_guess = [100, 10, 0.1]

    # Find the optimal Kp, Ki, Kd values using optimization
    result = minimize(
        objective_function,
        initial_guess,
        args=(
            cooling_rate,
            heating_rate,
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
        target_temp,
        simulation_time,
        dt,
    )

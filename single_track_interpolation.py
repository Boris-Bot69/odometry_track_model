import matplotlib.pyplot as plt
import numpy as np

# Constants
wheel_contact_points_x = 2.0
r_b = 3.25  # Wheelbase
vFL, vFR = 3.0, 3.0  # Front left and right wheel speeds
vRL, vRR = 3.0, 3.0  # Rear left and right wheel speeds

# Average speed of rear wheels
v = (vRL + vRR) / 2

# Time settings
dt = 0.1  # Time step

# Initialize state variables
x, y, theta = 0.0, 0.0, 0.0
beta_R = 0.0  # Initial rear wheel angle (no steering)
beta_F = 1.0  # Initial front wheel angle for turning

# Arrays to store the simulation results
x_list, y_list, theta_list, r_RM_list, beta_F_list, beta_R_list, omega_list, velocity_list = [], [], [], [], [], [], [], []

# Simulation loop setup
turn_radius = 5.0  # Turning radius for quarter circle
turn_angle = np.pi / 2  # Quarter circle turn in radians
turn_duration = turn_angle * (turn_radius / v)  # Time to complete a quarter turn
straight_length = 3.0  # Length of straight movement
straight_duration = straight_length / v  # Time to move straight
transition_duration = 1.0  # Duration for the smooth transition
maneuvers = [('turn', turn_duration), ('transition_to_straight', transition_duration),
             ('straight', straight_duration), ('transition_to_turn', transition_duration)] * 2

# Cubic Hermite spline interpolation function
def cubic_hermite_interpolation(t, start, end, duration):
    t = t / duration  # Normalize time
    h00 = (1 + 2 * t) * (1 - t) ** 2
    h10 = t * (1 - t) ** 2
    h01 = t ** 2 * (3 - 2 * t)
    h11 = t ** 2 * (t - 1)
    m0 = 0  # Start tangent (zero for smooth transition)
    m1 = 0  # End tangent (zero for smooth transition)
    return h00 * start + h10 * m0 + h01 * end + h11 * m1

t = 0
while maneuvers:
    maneuver, duration = maneuvers.pop(0)
    start_beta_F = beta_F
    if maneuver == 'turn':
        end_beta_F = np.arctan((turn_radius - wheel_contact_points_x) / r_b)
    elif maneuver == 'transition_to_straight':
        end_beta_F = 0.0
        transition_start_time = t
    elif maneuver == 'transition_to_turn':
        end_beta_F = np.arctan((turn_radius - wheel_contact_points_x) / r_b)
        transition_start_time = t
    else:  # straight
        end_beta_F = 0.0

    end_time = t + duration
    while t < end_time:
        # Interpolate beta_F if in transition
        if 'transition' in maneuver:
            transition_time = t - transition_start_time
            beta_F = cubic_hermite_interpolation(transition_time, start_beta_F, end_beta_F, duration)
        else:
            beta_F = end_beta_F

            # Calculate omega
        omega = (v / r_b) * (np.cos(beta_R) * (np.tan(beta_F) - np.tan(beta_R)))
        omega_threshold = 1e-5  # Threshold for omega to avoid division by very small numbers
        if abs(omega) > omega_threshold:
           r_RM = v / omega
        else:
           r_RM = 1e10

        # Update vehicle state
        x += v * dt * np.cos(beta_R + theta + omega * dt / 2)
        y += v * dt * np.sin(beta_R + theta + omega * dt / 2)
        theta += omega * dt

        # Save the state
        x_list.append(x)
        y_list.append(y)
        theta_list.append(theta)
        r_RM_list.append(r_RM)
        beta_F_list.append(beta_F)
        beta_R_list.append(beta_R)
        omega_list.append(omega)
        velocity_list.append(v)

        t += dt  # Increment time

# Plotting the trajectory and other data
plt.figure(figsize=(18, 10))

plt.subplot(2, 3, 1)
plt.scatter(x_list[0], y_list[0], color='green', label='Start')
plt.scatter(x_list[-1], y_list[-1], color='red', label='End')
plt.plot(x_list, y_list, label='Car Trajectory')
plt.title("Simulated Car Trajectory")
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')

# Theta plot
plt.subplot(2, 3, 2)
plt.plot(np.arange(len(theta_list)) * dt, theta_list, label='Theta over Time', color='blue')
plt.title('Theta over Time')
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')
plt.grid(True)
plt.legend()

# Beta_F and Beta_R plot
plt.subplot(2, 3, 3)
plt.plot(np.arange(len(beta_F_list)) * dt, beta_F_list, label='Beta_F', color='orange')
plt.plot(np.arange(len(beta_R_list)) * dt, beta_R_list, label='Beta_R', color='purple')
plt.title('Beta_F and Beta_R over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.grid(True)
plt.legend()

# r_RM plot
plt.subplot(2, 3, 4)
plt.plot(np.arange(len(r_RM_list)) * dt, r_RM_list, label='r_RM over Time', color='red')
plt.title('r_RM over Time')
plt.xlabel('Time (s)')
plt.ylabel('r_RM (m)')
plt.grid(True)
plt.legend()

# Omega plot
plt.subplot(2, 3, 5)
plt.plot(np.arange(len(omega_list)) * dt, omega_list, label='Omega over Time', color='green')
plt.title('Omega over Time')
plt.xlabel('Time (s)')
plt.ylabel('Omega (rad/s)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

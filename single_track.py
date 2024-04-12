import matplotlib.pyplot as plt
import numpy as np

# Constants

# wheel contact points in x-direction
rFL = 2.0
rFX = 2.0
rRL = 2.0
rRR = 2.0

r_b = 2.7  # (rFL + rFX) / 2 - (rRL + rRR) / 2  # Wheelbase
# r_b = (rFL,x + rFX, x) / 2  -  (rRL,x + rRR,x) / 2


# single wheel speed
vFL = 3.0
vFR = 3.0
vRL = 2.0
vRR = 4.0

v = (vRL + vRR) / 2

total_time = 40.0
dt = 0.1

# Initialize state variables
x, y, theta = 0.0, 0.0, 0.0

# beta_F > beta_R
# optimal angles: beta_R = (0.0, 1.0)
# optimal angles: beta_F = (1.0, 1.5)
beta_R = 0.1
beta_F = 2.0

# Dynamically adjust beta_F to simulate different steering behaviors
# For this simulation, we'll assume beta_F is a function of time

# Arrays to store the simulation results
x_list, y_list, theta_list, r_RM_list, beta_F_list, beta_R_list, omega_list = [], [], [], [], [], [], []

# Simulation loop
for t in np.arange(0, total_time, dt):
    # Equations (8) result into (12)
    omega = (v / r_b) * (np.cos(beta_R) * (np.tan(beta_F) - np.tan(beta_R)))

    # equation (11)
    r_RM = v / omega

    # equation (14)
    beta_F = np.arctan(omega * r_b / v * np.cos(beta_R) + np.tan(beta_R))

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

# print(f"theta_k : {theta_list}\n")
# print(f"omega_k : {omega_list}\n")
# print(f"r_RM: {r_RM_list}")
# print(f"beta_F: {beta_F_list}")
# print(f"beta_R: {beta_R_list}")
# Plotting the trajectory
# Explicitly setting the aspect ratio to be equal
plt.figure(figsize=(18, 10))

# Creating a grid for plotting
gs = plt.GridSpec(1, 5, width_ratios=[3, 1, 1, 1, 1])

# Plotting the trajectory on the left side
plt.subplot(gs[0])
plt.plot(x_list, y_list, label='Car Trajectory')
plt.scatter(x_list[0], y_list[0], color='green', label='Start')
plt.scatter(x_list[-1], y_list[-1], color='red', label='End')
plt.title(f"Simulated Car Trajectory with {total_time} time step")
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.grid(True)
plt.gca().set_aspect('equal', adjustable='box')
plt.xlim(min(x_list) - 1, max(x_list) + 1)
plt.ylim(min(y_list) - 1, max(y_list) + 1)

# Plotting the other subplots on the right side
subplots_titles = ['Theta over Time', 'Omega over Time', 'r_RM over Time', 'Beta_R and Beta_F over Time']
subplots_data = [theta_list, omega_list, r_RM_list, [beta_R_list, beta_F_list]]
colors = ['blue', 'green', 'red', 'orange']

for i in range(4):
    plt.subplot(gs[i + 1])
    if i == 3:
        for j in range(2):
            plt.plot(np.arange(0, total_time, dt), subplots_data[i][j], label=subplots_titles[i], color=colors[j])
    else:
        plt.plot(np.arange(0, total_time, dt), subplots_data[i], label=subplots_titles[i], color=colors[i])
    plt.title(subplots_titles[i])
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.grid(True)
    plt.legend()

plt.tight_layout()
plt.show()

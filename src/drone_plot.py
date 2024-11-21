import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def plot_quadrotor(ax, x, y, z, yaw=0, size=1.0, arm_length=1.0, rotor_radius=0.2, motor_size=0.1, color='blue'):
    uav_elements = []  # To store all plot elements

    # Define arm endpoints relative to the center
    arm_offsets = np.array([
        [arm_length, 0, 0],  # Arm 1
        [-arm_length, 0, 0],  # Arm 2
        [0, arm_length, 0],  # Arm 3
        [0, -arm_length, 0],  # Arm 4
    ]) * size

    # Rotate arms by the yaw angle
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    rotated_offsets = arm_offsets @ rotation_matrix.T

    # Translate arms to the quadrotor's position
    arm_positions = rotated_offsets + np.array([x, y, z])

    # Draw the arms
    for i in range(4):
        arm_plot, = ax.plot(
            [x, arm_positions[i, 0]],  # X-coordinates
            [y, arm_positions[i, 1]],  # Y-coordinates
            [z, arm_positions[i, 2]],  # Z-coordinates
            color=color, linewidth=2
        )
        uav_elements.append(arm_plot)

    # Precompute rotor surface grid
    u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    rotor_x_base = rotor_radius * np.cos(u) * np.sin(v)
    rotor_y_base = rotor_radius * np.sin(u) * np.sin(v)
    rotor_z_base = rotor_radius * np.cos(v)

    # Draw the motors as spheres and rotors as circles
    for pos in arm_positions:
        # Draw motor
        motor = _plot_sphere(ax, pos[0], pos[1], pos[2], radius=motor_size, color='black', u=u, v=v)
        uav_elements.append(motor)

        # Draw rotor
        rotor_surface = ax.plot_surface(
            pos[0] + rotor_x_base, pos[1] + rotor_y_base, pos[2] + rotor_z_base,
            color='grey', alpha=0.6
        )
        uav_elements.append(rotor_surface)

    return uav_elements

def _plot_sphere(ax, x, y, z, radius=0.1, color='black', u=None, v=None):
    if u is None or v is None:
        u, v = np.mgrid[0:2 * np.pi:20j, 0:np.pi:10j]
    xs = x + radius * np.cos(u) * np.sin(v)
    ys = y + radius * np.sin(u) * np.sin(v)
    zs = z + radius * np.cos(v)
    return ax.plot_surface(xs, ys, zs, color=color, alpha=0.9)

# Example Usage
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([0, 10])

# Plot a quadrotor UAV at position (x=0, y=0, z=5)
plot_quadrotor(ax, x=0, y=0, z=5, yaw=np.pi / 6, size=1.0, arm_length=1.5, rotor_radius=0.3, motor_size=0.15)

# Set axis labels and show plot
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()
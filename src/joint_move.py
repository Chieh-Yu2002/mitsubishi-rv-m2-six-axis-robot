import numpy as np
import matplotlib.pyplot as plt
import os
from src import calculate_forward_kinematics, calculate_inverse_kinematics

# Constants
Tacc = 0.2
T = 0.5
r = (T - Tacc) / T

# End points and their transformation matrices
A = np.array([
    [0, 1, 0, 0.4],
    [0, 0, -1, 0.2],
    [-1, 0, 0, -0.3],
    [0, 0, 0, 1]
])

B = np.array([
    [0, 0, -1, 0.4],
    [-1, 0, 0, -0.3],
    [0, 1, 0, 0.1],
    [0, 0, 0, 1]
])

C = np.array([
    [1, 0, 0, 0.3],
    [0, -1, 0, 0.3],
    [0, 0, -1, 0.2],
    [0, 0, 0, 1]
])

# Joint angles
thetaA = np.array(calculate_inverse_kinematics(A)[0])
thetaB = np.array(calculate_inverse_kinematics(B)[0])
thetaC = np.array(calculate_inverse_kinematics(C)[0])

# Initialize motion arrays
theta_p = []
theta_v = []
theta_a = []

# Joint Move
for t in np.arange(-0.5, 0.5, 0.002):
    if t < -0.2:
        theta_p.append((thetaB - thetaA) * (t + 0.5) / T + thetaA)
        theta_v.append((thetaB - thetaA) / T)
        theta_a.append(np.zeros(6))
    elif -0.2 <= t <= 0.2:
        h = (t + Tacc) / (2 * Tacc)
        theta_p.append(
            (((thetaC - thetaB) * Tacc / T + ((thetaA + (thetaB - thetaA) * r) - thetaB)) *
             (2 - h) * h ** 2 - 2 * ((thetaA + (thetaB - thetaA) * r) - thetaB)) * h + 
             ((thetaA + (thetaB - thetaA) * r) - thetaB) + thetaB
        )
        theta_v.append(
            (((thetaC - thetaB) * Tacc / T + ((thetaA + (thetaB - thetaA) * r) - thetaB)) *
             (1.5 - h) * 2 * h ** 2 - ((thetaA + (thetaB - thetaA) * r) - thetaB)) / Tacc
        )
        theta_a.append(
            ((thetaC - thetaB) * Tacc / T + ((thetaA + (thetaB - thetaA) * r) - thetaB)) *
            (1 - h) * 3 * h / Tacc ** 2
        )
    elif t > 0.2:
        theta_p.append((thetaC - thetaB) * t / T + thetaB)
        theta_v.append((thetaC - thetaB) / T)
        theta_a.append(np.zeros(6))

# Convert lists to numpy arrays
theta_p = np.array(theta_p).T
theta_v = np.array(theta_v).T
theta_a = np.array(theta_a).T

# Function definitions of forward kinematics
def forward_kinematic(theta):
    return calculate_forward_kinematics(*np.radians(theta))

p = []
Xaxis = []
Zaxis = []

# Figure 1: 3D Path of Joint Move with Axis Labels
for i in theta_p.T:
    T = forward_kinematic(i)
    p.append(T[:3, 3])
    tempX = T @ np.array([0.1, 0, 0, 1])
    tempZ = T @ np.array([0, 0, 0.1, 1])
    Xaxis.append(tempX[:3])
    Zaxis.append(tempZ[:3])

p = np.array(p).T
Xaxis = np.array(Xaxis).T
Zaxis = np.array(Zaxis).T

# Save joint angles to a text file
output_file = "joint_move_angles.txt"
with open(output_file, "w") as file:
    # Write header
    file.write("t\tjoint1\tjoint2\tjoint3\tjoint4\tjoint5\tjoint6\tif out of range?\n")
    for i, angles in enumerate(theta_p.T):
        # Check if any joint angles are out of range
        out_of_range = any([
            not (-150 <= angles[0] <= 150),
            not (-30 <= angles[1] <= 100),
            not (-120 <= angles[2] <= 0),
            not (-110 <= angles[3] <= 110),
            not (-180 <= angles[4] <= 180),
            not (-180 <= angles[5] <= 180)
        ])
        # Format angles and write to file
        formatted_angles = "\t".join([f"{angle:.6f}" for angle in angles])
        file.write(f"{i}\t{formatted_angles}\t{out_of_range}\n")


# Plot results
# 1. 3D Path of Joint Move with Axis Labels
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Plot vectors indicating orientation (x-axis)
ax.quiver(
    p[0, :], p[1, :], p[2, :],
    Xaxis[0, :] - p[0, :],
    Xaxis[1, :] - p[1, :],
    Xaxis[2, :] - p[2, :],
    length=0.3,
    color='magenta'
)

# Plot vectors indicating orientation (z-axis)
ax.quiver(
    p[0, :], p[1, :], p[2, :],
    Zaxis[0, :] - p[0, :],
    Zaxis[1, :] - p[1, :],
    Zaxis[2, :] - p[2, :],
    length=0.3,
    color='cyan'
)

# Add axes for A, B, and C points
points = {"P1": A[:3, 3], "P2": B[:3, 3], "P3": C[:3, 3]}
for label, coord in points.items():
    ax.scatter(coord[0], coord[1], coord[2], color='red', s=40)
    ax.text(coord[0], coord[1], coord[2], f"{label}({coord[0]:.1f} {coord[1]:.1f} {coord[2]:.1f})")

# Set axis labels and title
ax.set_title('3D path of Joint Move')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
# plt.savefig(os.path.join("figure", "Joint Move - 3D Path.png"))
plt.show()


# 2. Joint Move - Angle
plt.figure(figsize=(12, 8))
joints = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"]
for i in range(6):
    plt.subplot(3, 2, i + 1)
    plt.plot(np.arange(0, theta_p.shape[1] * 0.002, 0.002), theta_p[i, :])
    plt.title(joints[i])
    plt.xlabel("time(sec)", fontsize=10)
    plt.ylabel("angle(degree)", fontsize=10)
plt.tight_layout()
plt.title("Joint Move - Angle")
# plt.savefig(os.path.join("figure", "Joint Move - Angle.png"))
plt.show()

# 3. Joint Move - Angular Velocity
plt.figure(figsize=(12, 8))
for i in range(6):
    plt.subplot(3, 2, i + 1)
    plt.plot(np.arange(0, theta_v.shape[1] * 0.002, 0.002), theta_v[i, :])
    plt.title(joints[i])
    plt.xlabel("time(sec)", fontsize=10)
    plt.ylabel("angular velocity(degree/sec)", fontsize=10)
plt.tight_layout()
plt.title("Joint Move - Angular Velocity")
# plt.savefig(os.path.join("figure", "Joint Move - Angular Velocity.png"))
plt.show()

# 4. Joint Move - Angular Acceleration
plt.figure(figsize=(12, 8))
for i in range(6):
    plt.subplot(3, 2, i + 1)
    plt.plot(np.arange(0, theta_a.shape[1] * 0.002, 0.002), theta_a[i, :])
    plt.title(joints[i])
    plt.xlabel("time(sec)", fontsize=10)
    plt.ylabel("angular acceleration(degree/sec^2)", fontsize=10)
plt.tight_layout()
plt.title("Joint Move - Angular Acceleration")
# plt.savefig(os.path.join("figure", "Joint Move - Angular Acceleration.png"))
plt.show()

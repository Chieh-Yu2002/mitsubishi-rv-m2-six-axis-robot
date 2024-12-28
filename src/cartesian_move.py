import numpy as np
import matplotlib.pyplot as plt
import os
from src import calculate_inverse_kinematics

# Constants
Tacc = 0.2
T = 0.5
r = (T - Tacc) / T

# End points
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

# Function definitions
def funD1(p1, p2):
    x = p1[:3, 0].T @ (p2[:3, 3] - p1[:3, 3])
    y = p1[:3, 1].T @ (p2[:3, 3] - p1[:3, 3])
    z = p1[:3, 2].T @ (p2[:3, 3] - p1[:3, 3])
    psi = np.arctan2(p1[:3, 1].T @ p2[:3, 2], p1[:3, 0].T @ p2[:3, 2])
    theta = np.arctan2(
        np.sqrt((p1[:3, 0].T @ p2[:3, 2])**2 + (p1[:3, 1].T @ p2[:3, 2])**2),
        p1[:3, 2].T @ p2[:3, 2]
    )
    phiS = (-np.sin(psi) * np.cos(psi) * (1 - np.cos(theta)) * p1[:3, 0].T @ p2[:3, 0]
            + (np.cos(psi)**2 * (1 - np.cos(theta)) + np.cos(theta)) * p1[:3, 1].T @ p2[:3, 0]
            - np.sin(psi) * np.sin(theta) * p1[:3, 2].T @ p2[:3, 0])
    phiC = (-np.sin(psi) * np.cos(psi) * (1 - np.cos(theta)) * p1[:3, 0].T @ p2[:3, 1]
            + (np.cos(psi)**2 * (1 - np.cos(theta)) + np.cos(theta)) * p1[:3, 1].T @ p2[:3, 1]
            - np.sin(psi) * np.sin(theta) * p1[:3, 2].T @ p2[:3, 1])
    phi = np.arctan2(phiS, phiC)
    return x, y, z, psi, theta, phi

# Compute the motion matrix Dr
def funDr(x, y, z, psi, theta, phi, r):
    Dr = np.zeros((4, 4))
    Dr[3, 3] = 1
    Dr[:3, 1] = np.array([
        -np.sin(r * phi) * (np.sin(psi)**2 * (1 - np.cos(r * theta)) + np.cos(r * theta)) + np.cos(r * phi) * np.sin(psi) * np.sin(r * theta),
        -np.sin(r * phi) * (-np.sin(psi) * np.cos(psi) * (1 - np.cos(r * theta))) + np.cos(r * phi) * (np.cos(psi)**2 * (1 - np.cos(r * theta)) + np.cos(r * theta)),
        -np.sin(r * phi) * (-np.cos(psi) * np.sin(r * theta)) + np.cos(r * phi) * (-np.sin(psi) * np.sin(r * theta))
    ])
    Dr[:3, 2] = np.array([
        np.cos(psi) * np.sin(r * theta),
        np.sin(psi) * np.sin(r * theta),
        np.cos(r * theta)
    ])
    Dr[:3, 3] = [r * x, r * y, r * z]
    Dr[:3, 0] = np.cross(Dr[:3, 1], Dr[:3, 2])
    return Dr

# Compute Ap
x, y, z, psi, theta, phi = funD1(A, B)
Dr = funDr(x, y, z, psi, theta, phi, r)
Ap = A @ Dr

# Compute motion matrices for B to Ap and B to C
XA, YA, ZA, psiA, thetaA, phiA = funD1(B, Ap)
XC, YC, ZC, psiC, thetaC, phiC = funD1(B, C)

delta_B = [XA, YA, ZA]
delta_C = [XC, YC, ZC]

# Check if the orientation change is greater than 90 degrees
if abs(np.rad2deg(psiC - psiA)) > 90:
    psiA = psiA + np.pi
    thetaA = -thetaA
    if psiA > np.pi:
        psiA = -2 * np.pi + psiA

# Generate trajectory
p = []
v = []
a = []
joint_theta = []
Xaxis = []
Zaxis = []

t_array = np.arange(0, 1 + 0.002, 0.002)

# Cartesian Move
for t in t_array - 0.5:
    if t < -0.2:
        h = (t + 0.5) / T
        Dr = funDr(x, y, z, psi, theta, phi, h)
        Dv = np.array([x / T, y / T, z / T, 0])
        joint = calculate_inverse_kinematics(A @ Dr)
        joint_theta.append(joint)
        p.append((A @ Dr)[:3, 3])
        v.append((A[:3, :3] @ Dv[:3]))
        a.append(np.zeros(3))
        tempX = A @ Dr @ np.array([0.1, 0, 0, 1])
        tempZ = A @ Dr @ np.array([0, 0, 0.1, 1])
        Xaxis.append(tempX[:3])
        Zaxis.append(tempZ[:3])
    elif -0.2 <= t <= 0.2:
        h = (t + Tacc) / (2 * Tacc)
        Dp = np.array([
            ((XC * Tacc / T + XA) * (2 - h) * h**2 - 2 * XA) * h + XA,
            ((YC * Tacc / T + YA) * (2 - h) * h**2 - 2 * YA) * h + YA,
            ((ZC * Tacc / T + ZA) * (2 - h) * h**2 - 2 * ZA) * h + ZA,
            (psiC - psiA) * h + psiA,
            ((thetaC * Tacc / T + thetaA) * (2 - h) * h**2 - 2 * thetaA) * h + thetaA,
            ((phiC * Tacc / T + phiA) * (2 - h) * h**2 - 2 * phiA) * h + phiA
        ])
        Dv = np.array([
            ((XC * Tacc / T + XA) * (1.5 - h) * 2 * h**2 - XA) / Tacc,
            ((YC * Tacc / T + YA) * (1.5 - h) * 2 * h**2 - YA) / Tacc,
            ((ZC * Tacc / T + ZA) * (1.5 - h) * 2 * h**2 - ZA) / Tacc,
            0
        ])
        Da = np.array([
            (XC * Tacc / T + XA) * (1 - h) * 3 * h / Tacc**2,
            (YC * Tacc / T + YA) * (1 - h) * 3 * h / Tacc**2,
            (ZC * Tacc / T + ZA) * (1 - h) * 3 * h / Tacc**2,
            0
        ])
        Dr = funDr(Dp[0], Dp[1], Dp[2], Dp[3], Dp[4], Dp[5], 1)
        joint = calculate_inverse_kinematics(B @ Dr)
        joint_theta.append(joint)
        p.append((B @ Dr)[:3, 3])
        v.append((B[:3, :3] @ Dv[:3]))
        a.append((B[:3, :3] @ Da[:3]))
        tempX = B @ Dr @ np.array([0.1, 0, 0, 1])
        tempZ = B @ Dr @ np.array([0, 0, 0.1, 1])
        Xaxis.append(tempX[:3])
        Zaxis.append(tempZ[:3])
    elif t > 0.2:
        h = t / T
        Dr = funDr(XC, YC, ZC, psiC, thetaC, phiC, h)
        Dv = np.array([XC / T, YC / T, ZC / T, 0])
        joint = calculate_inverse_kinematics(B @ Dr)
        joint_theta.append(joint)
        p.append((B @ Dr)[:3, 3])
        v.append((B[:3, :3] @ Dv[:3]))
        a.append(np.zeros(3))
        tempX = B @ Dr @ np.array([0.1, 0, 0, 1])
        tempZ = B @ Dr @ np.array([0, 0, 0.1, 1])
        Xaxis.append(tempX[:3])
        Zaxis.append(tempZ[:3])

p = np.array(p).T
v = np.array(v).T
a = np.array(a).T
Xaxis = np.array(Xaxis).T
Zaxis = np.array(Zaxis).T
joint_theta = np.array(joint_theta).T

# Save joint angles to a text file
output_file = "cartesian_joint_angles.txt"
with open(output_file, "w") as file:
    file.write("t\tjoint1\tjoint2\tjoint3\tjoint4\tjoint5\tjoint6\tif out of range?\n")
    for i, angles in enumerate(joint_theta.T):
        out_of_range = any([
            not -150 <= angles[0][0] <= 150,
            not -30 <= angles[0][1] <= 100,
            not -120 <= angles[0][2] <= 0,
            not -110 <= angles[0][3] <= 110,
            not -180 <= angles[0][4] <= 180,
            not -180 <= angles[0][5] <= 180
        ])
        # Ensure each angle is a scalar
        formatted_angles = "\t".join([f"{angle:.6f}" for angle in angles.flatten()])
        file.write(f"{i}\t{formatted_angles}\t{out_of_range}\n")

# Plotting results
# 1. 3D Path for Cartesian Move
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
ax.set_title('3D path of Cartesian Move')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
# plt.savefig(os.path.join("figure", "Cartesian Move - Path.png"))
plt.show()

# Cartesian Move - Position
plt.figure(figsize=(8, 7))
plt.subplot(3, 1, 1)
plt.plot(t_array, p[0], label='X Position')
plt.title('Position - X')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.subplot(3, 1, 2)
plt.plot(t_array, p[1], label='Y Position')
plt.title('Position - Y')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.subplot(3, 1, 3)
plt.plot(t_array, p[2], label='Z Position')
plt.title('Position - Z')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.tight_layout()
# plt.savefig(os.path.join("figure", "Cartesian Move - Position.png"))
plt.show()

# Cartesian Move - Velocity
plt.figure(figsize=(8, 7))
plt.subplot(3, 1, 1)
plt.plot(t_array, v[0], label='X Velocity')
plt.title('Velocity - X')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.subplot(3, 1, 2)
plt.plot(t_array, v[1], label='Y Velocity')
plt.title('Velocity - Y')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.subplot(3, 1, 3)
plt.plot(t_array, v[2], label='Z Velocity')
plt.title('Velocity - Z')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.tight_layout()
# plt.savefig(os.path.join("figure", "Cartesian Move - Velocity.png"))
plt.show()

# Cartesian Move - Acceleration
plt.figure(figsize=(8, 7))
plt.subplot(3, 1, 1)
plt.plot(t_array, a[0], label='X Acceleration')
plt.title('Acceleration - X')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.subplot(3, 1, 2)
plt.plot(t_array, a[1], label='Y Acceleration')
plt.title('Acceleration - Y')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.subplot(3, 1, 3)
plt.plot(t_array, a[2], label='Z Acceleration')
plt.title('Acceleration - Z')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.tight_layout()
# plt.savefig(os.path.join("figure", "Cartesian Move - Acceleration.png"))
plt.show()

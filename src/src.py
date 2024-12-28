import numpy as np

# Function definitions for checking joint ranges
def check_joint_ranges(theta_solutions):
    """
    Check whether each set of joint angles is out of range
    """
    # Normalize angle to be within [-181, 181]
    def normalize_angle(a):
        """Normalize angle to be within [-181, 181]."""
        a = float(a)
        if a > 181:
            return a - 360
        elif a < -181:
            return a + 360
        return a

    # Check if a set of joint angles is within valid range
    def is_valid(theta):
        """Check if a set of joint angles is within valid range."""
        limits = [
            (-150, 150),  # theta1
            (-30, 100),   # theta2
            (-120, 0),    # theta3
            (-110, 110),  # theta4
            (-180, 180),  # theta5
            (-180, 180),  # theta6
        ]
        return all(
            low <= normalize_angle(theta[i]) <= high and not np.isnan(theta[i])
            for i, (low, high) in enumerate(limits)
        )

    # Filter and normalize valid solutions in one step
    return [
        [normalize_angle(a) for a in theta]
        for theta in theta_solutions
        if is_valid(theta)
    ]



def calculate_inverse_kinematics(A):
    # Extract positions px, py, pz from matrix A
    px = A[0, 3]
    py = A[1, 3]
    pz = A[2, 3]

    # Robot arm parameters
    a1 = np.float64(0.12)
    a2 = np.float64(0.25) 
    a3 = np.float64(0.26)

    # Calculate theta1 in 2 solutions
    theta1_options = [np.arctan2(py, px), np.arctan2(py, px) - np.pi]

    # Calculate R1 and R2
    r1 = np.cos(theta1_options[0])*px + np.sin(theta1_options[0])*py - a1
    r2 = np.cos(theta1_options[1])*px + np.sin(theta1_options[1])*py - a1
    s = -pz

    R1 = np.sqrt(r1**2 + s**2)
    R2 = np.sqrt(r2**2 + s**2)
    # Calculate cosalpha1 and cosalpha2
    cosalpha1 = np.clip((R1**2 + a2**2 - a3**2) / (2 * R1 * a2), -1.0, 1.0)
    cosalpha2 = np.clip((R2**2 + a2**2 - a3**2) / (2 * R2 * a2), -1.0, 1.0)

    alpha1 = np.arccos(cosalpha1)
    alpha2 = np.arccos(cosalpha2)

    beta1 = np.arccos(np.clip((R1**2 + a3**2 - a2**2) / (2 * R1 * a3), -1.0, 1.0))
    beta2 = np.arccos(np.clip((R2**2 + a3**2 - a2**2) / (2 * R2 * a3), -1.0, 1.0))

    # Calculate theta2 and theta3 for the first set of theta_options_1 and the second set of theta_options_2
    theta_options_1 = [[theta1_options[0], np.arctan2(s, r1) - np.arctan2(np.sqrt(1 - cosalpha1**2), cosalpha1), \
                        alpha1 + beta1],
                        [theta1_options[0], np.arctan2(s, r1) - np.arctan2(-np.sqrt(1 - cosalpha1**2), cosalpha1), \
                        -alpha1 - beta1]]

    theta_options_2 = [[theta1_options[1], np.arctan2(s, r2) - np.arctan2(np.sqrt(1 - cosalpha2**2), cosalpha2), \
                        alpha2 + beta2],
                        [theta1_options[1], np.arctan2(s, r2) - np.arctan2(-np.sqrt(1 - cosalpha2**2), cosalpha2), \
                        -alpha2 - beta2]]

    solutions = []

    # Extract rotation matrix
    ax = A[0, 2]
    ay = A[1, 2]
    az = A[2, 2]
    ox = A[0, 1]
    oy = A[1, 1]
    oz = A[2, 1]
    nx = A[0, 0]
    ny = A[1, 0]
    nz = A[2, 0]

    new_theta_options_1 = []
    new_theta_options_2 = []

    # Calculate theta4 for the first set of theta_options_1
    for theta123 in theta_options_1:
        numerator_theta4 = -np.cos(theta123[0]) * np.sin(theta123[1] + theta123[2]) * ax \
                          - np.sin(theta123[0]) * np.sin(theta123[1] + theta123[2]) * ay \
                          - np.cos(theta123[1] + theta123[2]) * az
        denominator_theta4 = np.cos(theta123[0]) * np.cos(theta123[1] + theta123[2]) * ax \
                          + np.sin(theta123[0]) * np.cos(theta123[1] + theta123[2]) * ay \
                          - np.sin(theta123[1] + theta123[2]) * az
        theta4_1 = np.arctan2(numerator_theta4, denominator_theta4)
        theta4_2 = theta4_1 + np.pi if theta4_1 < 0 else theta4_1 - np.pi
        new_theta_options_1.append([theta123[0], theta123[1], theta123[2], theta4_1])
        new_theta_options_1.append([theta123[0], theta123[1], theta123[2], theta4_2])

    # Calculate theta4 for the second set of theta_options_2
    for theta123 in theta_options_2:
        numerator_theta4 = -np.cos(theta123[0]) * np.sin(theta123[1] + theta123[2]) * ax \
                          - np.sin(theta123[0]) * np.sin(theta123[1] + theta123[2]) * ay \
                          - np.cos(theta123[1] + theta123[2]) * az
        denominator_theta4 = np.cos(theta123[0]) * np.cos(theta123[1] + theta123[2]) * ax \
                          + np.sin(theta123[0]) * np.cos(theta123[1] + theta123[2]) * ay \
                          - np.sin(theta123[1] + theta123[2]) * az
        theta4_1 = np.arctan2(numerator_theta4, denominator_theta4)
        theta4_2 = theta4_1 + np.pi if theta4_1 < 0 else theta4_1 - np.pi
        new_theta_options_2.append([theta123[0], theta123[1], theta123[2], theta4_1])
        new_theta_options_2.append([theta123[0], theta123[1], theta123[2], theta4_2])

    # Calculate theta5 and theta6 for the first set of theta_options_1
    for theta1234 in new_theta_options_1:
        numerator_theta5 = ax * np.cos(theta1234[0]) * np.cos(theta1234[1] + theta1234[2] + theta1234[3]) \
                          + ay * np.sin(theta1234[0]) * np.cos(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - az * np.sin(theta1234[1] + theta1234[2] + theta1234[3])
        denominator_theta5 = -ax * np.sin(theta1234[0]) + ay * np.cos(theta1234[0])
        theta5 = np.arctan2(numerator_theta5, denominator_theta5)   

        numerator_theta6 = -nx * np.cos(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - ny * np.sin(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - nz * np.cos(theta1234[1] + theta1234[2] + theta1234[3])
        denominator_theta6 = -ox * np.cos(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - oy * np.sin(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - oz * np.cos(theta1234[1] + theta1234[2] + theta1234[3])
        theta6 = np.arctan2(numerator_theta6, denominator_theta6)

        angles = [theta1234[0], theta1234[1], theta1234[2],theta1234[3], theta5, theta6]
        angles = [np.degrees(angle) for angle in angles]
        solutions.append(angles)

    # Calculate theta5 and theta6 for the second set of theta_options_2
    for theta1234 in new_theta_options_2:
        numerator_theta5 = ax * np.cos(theta1234[0]) * np.cos(theta1234[1] + theta1234[2] + theta1234[3]) \
                          + ay * np.sin(theta1234[0]) * np.cos(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - az * np.sin(theta1234[1] + theta1234[2] + theta1234[3])
        denominator_theta5 = -ax * np.sin(theta1234[0]) + ay * np.cos(theta1234[0])
        theta5 = np.arctan2(numerator_theta5, denominator_theta5)

        numerator_theta6 = -nx * np.cos(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - ny * np.sin(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - nz * np.cos(theta1234[1] + theta1234[2] + theta1234[3])
        denominator_theta6 = -ox * np.cos(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - oy * np.sin(theta1234[0]) * np.sin(theta1234[1] + theta1234[2] + theta1234[3]) \
                          - oz * np.cos(theta1234[1] + theta1234[2] + theta1234[3])
        theta6 = np.arctan2(numerator_theta6, denominator_theta6)

        angles = [theta1234[0], theta1234[1], theta1234[2], theta1234[3], theta5, theta6]
        angles = [np.degrees(angle) for angle in angles]
        solutions.append(angles)

    valid_solutions = check_joint_ranges(solutions)

    return valid_solutions

# Function definitions for forward kinematics
def calculate_forward_kinematics(theta1, theta2, theta3, theta4, theta5, theta6):
    # Define link lengths
    a1 = np.float64(0.12)
    a2 = np.float64(0.250)
    a3 = np.float64(0.260)

    # Calculate transformation matrices
    A1 = np.array([[np.cos(theta1), 0.0, -np.sin(theta1), a1*np.cos(theta1)],
                [np.sin(theta1), 0.0, np.cos(theta1), a1*np.sin(theta1)],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
    A2 = np.array([[np.cos(theta2), -np.sin(theta2), 0.0, a2*np.cos(theta2)],
                [np.sin(theta2), np.cos(theta2), 0.0, a2*np.sin(theta2)],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
    A3 = np.array([[np.cos(theta3), -np.sin(theta3), 0.0, a3*np.cos(theta3)],
                [np.sin(theta3), np.cos(theta3), 0.0, a3*np.sin(theta3)],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
    A4 = np.array([[np.cos(theta4), 0.0, -np.sin(theta4), 0.0],
                [np.sin(theta4), 0.0, np.cos(theta4), 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
    A5 = np.array([[np.cos(theta5), 0.0, np.sin(theta5), 0.0],
                [np.sin(theta5), 0.0, -np.cos(theta5), 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
    A6 = np.array([[np.cos(theta6), -np.sin(theta6), 0.0, 0.0],
                [np.sin(theta6), np.cos(theta6), 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]])
    
    # Calculate the final transformation matrix
    T = A1 @ A2 @ A3 @ A4 @ A5 @ A6
    return T

# Function definitions for calculating angles and positions
def calculate_angles_and_positions(T):
    # Extract positions px, py, pz from matrix T
    px = T[0, 3]
    py = T[1, 3]
    pz = T[2, 3]

    # Calculate phi, theta, and psi
    phi = np.arctan2(T[1, 2], T[0, 2])
    theta = np.arctan2(np.cos(phi)*T[0, 2] + np.sin(phi)*T[1, 2], T[2, 2])
    psi = np.arctan2(-np.sin(phi)*T[0, 0] + np.cos(phi)*T[1, 0], -np.sin(phi)*T[0, 1] + np.cos(phi)*T[1, 1])

    # Convert angles from radians to degrees
    phi = np.degrees(phi)
    theta = np.degrees(theta)
    psi = np.degrees(psi)

    return px, py, pz, phi, theta, psi


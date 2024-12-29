# Path Planning for Mitsubishi RV-M2 Robot Manipulator

This repository contains the implementation of path planning for the **Mitsubishi RV-M2** six-joint robot manipulator. The implementation includes **joint move** and **Cartesian move**, using custom forward and inverse kinematics. Additionally, the project explores the variations in key parameters such as six-axis variables, speed, and acceleration.

## Project Overview

The goal of this project is to plan and visualize the paths for the robot manipulator, focusing on:
- **Utilizing the DH table to establish forward kinematics**, allowing the computation of the end-effector's pose in the Cartesian space.
- **Using analytical solutions to compute inverse kinematics**, where:
  - **Geometric method** is applied to solve for the first three joint angles (θ₁, θ₂, θ₃).
  - **Algebraic method** is used for the last three joint angles (θ₄, θ₅, θ₆).
- **Joint Move**: Planning the Cartesian path corresponding to a chosen joint configuration.
- **Cartesian Move**: Planning a feasible joint path for a given Cartesian trajectory.

Each move consists of:
- **Straight Line Portion**
- **Transition Portion**: Ensures position, velocity, and acceleration continuities.

---

## Robot Kinematic Table

| Joint | d (in) | a (mm)  | α (°)  | θ (°)    |
|-------|--------|---------|--------|----------|
| 1     | 0      | 120     | -90    | Variable |
| 2     | 0      | 250     | 0      | Variable |
| 3     | 0      | 260     | 0      | Variable |
| 4     | 0      | 0       | -90    | Variable |
| 5     | 0      | 0       | 90     | Variable |
| 6     | 0      | 0       | 0      | Variable |

**Joint Range Constraints**:
- −150° ≤ θ₁ ≤ 150°
- −30° ≤ θ₂ ≤ 100°
- −120° ≤ θ₃ ≤ 0°
- −110° ≤ θ₄ ≤ 110°
- −180° ≤ θ₅ ≤ 180°
- −180° ≤ θ₆ ≤ 180°

---

## Implementation Details

### Forward Kinematics
The forward kinematics was derived based on the Denavit-Hartenberg (DH) table for the Mitsubishi RV-M2 manipulator. This includes:
- Translation and rotation matrices for each joint.
- Computing the pose of the end-effector in the Cartesian space.

### Inverse Kinematics
The inverse kinematics was solved using a hybrid approach:
- **Geometric Method**: For the first three joints (θ₁, θ₂, θ₃).
- **Algebraic Method**: For the last three joints (θ₄, θ₅, θ₆).
- The solutions are organized into an **inverse kinematics table** for easy reference and computation.

### Path Planning
- **Joint Move**: Generates positions, velocities, and accelerations over time for the end-effector based on joint configurations.
- **Cartesian Move**: Computes transformation matrices and motion profiles for intermediate points and derives joint angles.

---

## Files in This Repository

| File Name           | Description                                                                                     |
|---------------------|-------------------------------------------------------------------------------------------------|
| `joint_move.py`     | Implements joint move path planning, including trajectory generation and motion visualization.  |
| `cartesian_move.py` | Implements Cartesian move path planning and generates motion profiles.                          |
| `src.py`            | Entry point script to execute both joint and Cartesian moves.                                   |

---

## How to Run

1. Place all files (`joint_move.py`, `cartesian_move.py`, `src.py`) in the same folder.
2. Install required libraries:
   ```bash
   pip install numpy matplotlib
   ```
3. Execute the main script:
   ```bash
   python src.py
   ```
4. Results:
   - Visualization: Graphs of 3D trajectories, joint angles, velocities, and accelerations.
   - Output Files: `joint_move_angles.txt` and `cartesian_joint_angles.txt`.

---

## Results
- **Visualization**:
  - Joint move and Cartesian move trajectories.
    
    <img src="https://github.com/user-attachments/assets/bcf24b76-983c-4f02-b30d-a4f93cdfc510" alt="Joint Move - Trajectory" width="600"/>
    <img src="https://github.com/user-attachments/assets/8edb2d12-97fd-4b2a-a887-dcae141fd3da" alt="Cartesian Move - Trajectory" width="600"/>

  - Motion profiles for joint angles and end-effector.
    <img src="https://github.com/user-attachments/assets/9e8376d7-3e09-4ce0-bd09-1d7e8714a575" alt="Joint Move - Angle" width="600"/>
    <img src="https://github.com/user-attachments/assets/289d4d5f-2a81-4563-840c-23b63861337c" alt="Joint Move - Angular Acceleration" width="600"/>
    <img src="https://github.com/user-attachments/assets/2170ddbc-1d28-483e-8ffb-9b265df5d816" alt="Joint Move - Angular Velocity" width="600"/>
    <img src="https://github.com/user-attachments/assets/08b801f6-7666-43a9-b2f2-9dd204ebe36b" alt="Cartesian Move - Position" width="600"/>
    <img src="https://github.com/user-attachments/assets/602002a3-fda5-494d-9b23-79a305eaa545" alt="Cartesian Move - Velocity" width="600"/>
    <img src="https://github.com/user-attachments/assets/79e48797-66bf-47a6-ba4f-e104a60086f4" alt="Cartesian Move - Acceleration" width="600"/>



---

## Requirements
- Python 3.8+
- Libraries:
  - `numpy`
  - `matplotlib`
---

## License
This project is released under the MIT License.

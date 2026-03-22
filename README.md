# planar_robot_2d

C++ project for modeling and simulating a 2-DOF planar robot arm.

The current codebase includes:

- Forward and inverse kinematics for a 2-link planar manipulator
- A rigid-body dynamics model with mass, Coriolis, and gravity terms
- Total energy computation
- Time integration of the robot state using a fourth-order Runge-Kutta (RK4) simulator
- A simple console demo in `main.cc`

## Project Structure

```text
.
|-- CMakeLists.txt
|-- include/
|   |-- robot_model.hpp
|   `-- simulator.hpp
`-- src/
    |-- main.cc
    |-- robot_model.cc
    `-- simulator.cc
```

## Build Requirements

- CMake 3.10 or newer
- A C++17-compatible compiler
- Eigen3

On Ubuntu or Debian-based systems, Eigen can typically be installed with:

```bash
sudo apt install libeigen3-dev
```

## Build

From the repository root:

```bash
cmake -S . -B build
cmake --build build
```

This generates the executable:

```bash
./build/planar_robot
```

## Current Demo

The program in `src/main.cc` currently:

1. Creates a 2-link robot with:
   - link lengths `l1 = 1.0`, `l2 = 1.0`
   - link masses `m1 = 1.0`, `m2 = 1.0`
2. Creates a simulator with time step `dt = 1e-4`
3. Starts from:
   - joint angles `q = [0, 0]`
   - joint velocities `qdot = [0, 0]`
4. Applies zero joint torque `tau = [0, 0]`
5. Simulates `100000` steps
6. Prints the state and total energy every `10000` steps

This makes the current executable a useful baseline for checking dynamic behavior and energy consistency of the simulator.

## Main Components

### `RobotModel`

Declared in `include/robot_model.hpp` and implemented in `src/robot_model.cc`.

Responsibilities:

- Store the robot parameters
- Compute end-effector position with forward kinematics
- Solve inverse kinematics for reachable Cartesian targets
- Compute link center-of-mass positions
- Compute the mass matrix `M(q)`
- Compute Coriolis/centrifugal effects `C(q, qdot)`
- Compute gravity terms `G(q)`
- Compute forward dynamics
- Compute total mechanical energy

The model assumes:

- A planar 2-link serial arm
- Uniform links, with center of mass at half the link length
- Link inertia about each link center
- Gravity with magnitude `9.81 m/s^2`

## Robot Dynamics

The equations of motion used by the code follow the standard rigid-body manipulator form:

```math
M(q)\ddot{q} + C(q,\dot{q}) + G(q) = \tau
```

where:

- `q = [q1, q2]^T` is the joint-angle vector
- `qdot = [q1dot, q2dot]^T` is the joint-velocity vector
- `qddot = [q1ddot, q2ddot]^T` is the joint-acceleration vector
- `M(q)` is the mass matrix
- `C(q, qdot)` is the Coriolis/centrifugal vector
- `G(q)` is the gravity vector
- `tau` is the applied joint-torque vector

The simulator computes forward dynamics as:

```math
\ddot{q} = M(q)^{-1}\left(\tau - C(q,\dot{q}) - G(q)\right)
```

For the current 2-link model, the implemented mass matrix is:

```math
M(q) =
\begin{bmatrix}
I_1 + I_2 + m_1 l_{c1}^2 + m_2 \left(l_1^2 + l_{c2}^2 + 2 l_1 l_{c2}\cos q_2\right) &
I_2 + m_2 \left(l_{c2}^2 + l_1 l_{c2}\cos q_2\right) \\
I_2 + m_2 \left(l_{c2}^2 + l_1 l_{c2}\cos q_2\right) &
I_2 + m_2 l_{c2}^2
\end{bmatrix}
```

The Coriolis/centrifugal vector is:

```math
C(q,\dot{q}) =
\begin{bmatrix}
-h \dot{q}_2 \left(2\dot{q}_1 + \dot{q}_2\right) \\
h \dot{q}_1^2
\end{bmatrix},
\qquad
h = m_2 l_1 l_{c2}\sin q_2
```

The gravity vector is:

```math
G(q) =
\begin{bmatrix}
\left(m_1 l_{c1} + m_2 l_1\right) g \cos q_1 + m_2 l_{c2} g \cos(q_1 + q_2) \\
m_2 l_{c2} g \cos(q_1 + q_2)
\end{bmatrix}
```

In this codebase:

- `l1`, `l2` are the link lengths
- `lc1 = l1 / 2` and `lc2 = l2 / 2` are the center-of-mass locations
- `I1 = (1/12) m1 l1^2` and `I2 = (1/12) m2 l2^2` assume uniform slender links

These expressions are implemented in [src/robot_model.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/robot_model.cc).

### `Simulator`

Declared in `include/simulator.hpp` and implemented in `src/simulator.cc`.

Responsibilities:

- Store the simulation time step
- Advance the robot state using RK4 integration
- Track simulation time through the `RobotState` struct

`RobotState` contains:

- `q`: joint angles
- `qdot`: joint velocities
- `time`: simulation time

## Notes on Inverse Kinematics

The current inverse kinematics implementation:

- Rejects targets outside the manipulator workspace
- Rejects targets with `x < 0`
- Selects a solution whose first joint angle stays within `[-pi/2, pi/2]`

That means the current solver is intentionally restricted and does not yet expose all mathematically valid elbow-up and elbow-down solutions.

## License

This repository includes a `LICENSE` file at the project root.
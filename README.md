# planar_robot_2d

C++ simulation of a 2-DOF planar robot arm with inverse-kinematics-based trajectory tracking, joint-space impedance control, and simple environment interaction models.

The current codebase supports:

- Forward and inverse kinematics for a 2-link planar manipulator
- Jacobian computation for mapping Cartesian forces to joint torques
- Rigid-body dynamics with mass, Coriolis, and gravity terms
- 4th order Runge-Kutta time integration
- Scenario-based motion generation
- A stiff wall contact model
- A time-based external disturbance model
- Signal logging to timestamped files in `data/`

## Contents

- [Project Structure](#project-structure)
- [Build Requirements](#build-requirements)
- [Build And Run](#build-and-run)
- [Current Simulation Setup](#current-simulation-setup)
- [Scenarios And Environment Interaction](#scenarios-and-environment-interaction)
- [Control](#control)
- [Robot Model](#robot-model)
- [Numerical Integration](#numerical-integration)
- [Logging](#logging)
- [License](#license)

## Project Structure

```text
.
|-- CMakeLists.txt
|-- README.md
|-- include/
|   |-- controller.hpp
|   |-- logger.hpp
|   |-- robot_model.hpp
|   |-- scenarios.hpp
|   `-- simulator.hpp
`-- src/
    |-- controller.cc
    |-- logger.cc
    |-- main.cc
    |-- robot_model.cc
    |-- scenarios.cc
    `-- simulator.cc
```

## Build Requirements

- CMake 3.10 or newer
- A C++17-compatible compiler
- Eigen3

On Ubuntu or Debian-based systems:

```bash
sudo apt install libeigen3-dev
```

On macOS with homebrew
```bash
brew install eigen
```

## Build And Run

From the repository root:

```bash
cmake -S . -B build
cmake --build build
./build/planar_robot
```

## Current Simulation Setup

The executable in [src/main.cc](src/main.cc) currently:

- Creates a 2-link robot with `l1 = 1.0`, `l2 = 1.0`
- Uses link masses `m1 = 1.0`, `m2 = 1.0`
- Uses a fixed simulation step `dt = 1e-4`
- Uses a joint-space PD controller with gravity compensation
- Selects one scenario through `kScenarioMode`
- Logs signals every `1e-2` seconds

The active scenario is chosen in `main.cc` with:

```cpp
constexpr ScenarioMode kScenarioMode = ScenarioMode::FREE_SPACE_MOTION;
```

To run a different case, change that line to:

- `ScenarioMode::FREE_SPACE_MOTION`
- `ScenarioMode::STIFF_ENVIRONMENT`
- `ScenarioMode::EXTERNAL_DISTURBANCE`

## Scenarios And Environment Interaction

Scenario configuration is centralized in [include/scenarios.hpp](include/scenarios.hpp) and [src/scenarios.cc](src/scenarios.cc).

Each scenario is described by a `ScenarioConfig` containing:

- Initial joint angles `initial_q`
- Initial joint velocities `initial_qdot`
- Initial desired joint angles `initial_q_d`
- Initial desired joint velocities `initial_qdot_d`
- Initial end-effector position `initial_xy`
- Initial desired end-effector position `initial_xy_d`
- A desired end-effector trajectory function

The factory function

```cpp
ScenarioConfig makeScenarioConfig(ScenarioMode mode, const RobotModel& robot);
```

is the single place where scenario-specific initialization is defined.

### Free Space Motion

`runFreeSpaceMotion(double time_sec)` commands the end-effector to draw a rectangle:

```text
(1.0, 0.0) -> (1.5, 0.0) -> (1.5, 0.5) -> (1.0, 0.5) -> (1.0, 0.0)
```

The motion lasts `4` seconds, with each leg taking about `1` second.

### Stiff Environment

`runStiffEnvironment(double time_sec)` commands the end-effector to move along the x-axis:

```text
(0.5, 0.0) -> (1.5, 0.0) -> (0.5, 0.0)
```

The outward motion takes `2` seconds and the return motion takes `2` seconds.

When this scenario is active, the simulator also applies a wall contact force. The environment model is implemented in [src/simulator.cc](src/simulator.cc).

The wall is currently modeled as a unilateral linear spring at:

```text
x = 1.2
```

If the end-effector penetrates the wall, the simulator applies the Cartesian contact force:

```math
F_{\text{wall},x} = -k_{\text{wall}} (x - x_{\text{wall}})
```

with:

- `x_wall = 1.2`
- `k_wall = 10000`

No contact force is applied when `x <= 1.2`.

The Cartesian wall force is mapped into joint space in `main.cc` using:

```math
\tau_{\text{wall}} = J(q)^T F_{\text{wall}}
```

### External Disturbance

`runExternalDisturbance(double time_sec)` keeps the desired end-effector position fixed at:

```text
(1.0, 0.0)
```

When this scenario is active, the simulator applies a time-dependent external force disturbance. The disturbance model is also implemented in [src/simulator.cc](src/simulator.cc).

The external disturbance is currently a downward Cartesian force applied between `t = 1.0 s` and `t = 1.5 s`:

```text
F = [0, -20]
```

This force is also converted into joint torques using `J(q)^T F`.

## Control

The controller is implemented in [src/controller.cc](src/controller.cc).

It uses joint-space PD control with gravity compensation:

```math
\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + G(q)
```

The commanded torque is then saturated elementwise by `tau_max`.

In `main.cc`, Cartesian references are converted to joint references with inverse kinematics, and desired joint velocity is approximated numerically from successive desired joint positions.

## Robot Model

The robot model is declared in [include/robot_model.hpp](include/robot_model.hpp) and implemented in [src/robot_model.cc](src/robot_model.cc).

Responsibilities:

- Store the robot parameters
- Compute end-effector position with forward kinematics
- Solve inverse kinematics for reachable Cartesian targets
- Compute the end-effector Jacobian
- Compute link center-of-mass positions
- Compute the mass matrix `M(q)`
- Compute Coriolis and centrifugal effects `C(q, qdot)`
- Compute gravity terms `G(q)`
- Compute forward dynamics
- Compute total mechanical energy

The equations of motion follow:

```math
M(q)\ddot{q} + C(q,\dot{q}) + G(q) = \tau
```

and the forward dynamics are computed as:

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

The Coriolis and centrifugal vector is:

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

The model assumes:

- A planar 2-link serial arm
- Uniform links, with center of mass at half the link length
- Link inertia about each link center
- Gravity with magnitude `9.81 m/s^2`

## Numerical Integration

The simulator advances the robot state with fourth-order Runge-Kutta integration in [src/simulator.cc](src/simulator.cc).

`RobotState` stores:

- `q`: joint angles
- `qdot`: joint velocities
- `time`: current simulation time

`DesiredRobotState` stores:

- `q_d`: desired joint angles
- `qdot_d`: desired joint velocities
- `q_d_prev`: previous desired joint angles for numerical differentiation

For the state-space system

```math
\dot{q} = qdot, \qquad \dot{qdot} = f(q, qdot, \tau)
```

the RK4 integrator evaluates four slope estimates over one time step `h = dt`:

```math
k_1 = f(x_n, t_n)
```

```math
k_2 = f\left(x_n + \frac{h}{2}k_1, t_n + \frac{h}{2}\right)
```

```math
k_3 = f\left(x_n + \frac{h}{2}k_2, t_n + \frac{h}{2}\right)
```

```math
k_4 = f(x_n + h k_3, t_n + h)
```

and combines them as:

```math
x_{n+1} = x_n + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)
```

In this project, that is applied separately to:

- `q`, using the current joint velocity as the slope
- `qdot`, using forward dynamics to compute joint acceleration

Compared with simple Euler integration, RK4 gives better accuracy and stability for the same time step, especially when the robot dynamics become nonlinear during fast motion or contact.

## Logging

Each run creates a timestamped log file in `data/`.

The logger is declared in [include/logger.hpp](include/logger.hpp) and implemented in [src/logger.cc](src/logger.cc).

To use it:

1. Include the header:

```cpp
#include "logger.hpp"
```

2. Create a logger with an output folder and column names:

```cpp
SignalLogger logger(
    "data",
    {"time", "q1", "q2", "q1_d", "q2_d", "x", "y", "tau1", "tau2", "xy_d_x", "xy_d_y"});
```

3. Write rows wherever you want to record data:

```cpp
logger.writeRow({state.time, q1_deg, q2_deg, q1_d_deg, q2_d_deg,
                 xy(0), xy(1), tau(0), tau(1), xy_d(0), xy_d(1)});
```

4. Run the program:

```bash
./build/planar_robot
```

Each run creates a new space-delimited file with a comment-style header, for example:

```text
data/simulation_20260325_122736_840.txt
```

The current logger records:

```text
time q1 q2 q1_d q2_d x y tau1 tau2 xy_d_x xy_d_y
```

The logger checks that each row matches the declared number of columns and throws an error if the widths do not match.

This makes it easy to compare:

- actual vs desired joint motion
- actual vs desired end-effector position
- controller effort over time

## License

This repository includes a `LICENSE` file at the project root.

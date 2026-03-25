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

The executable in [src/main.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/main.cc) currently:

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

## Scenario System

Scenario configuration is centralized in [include/scenarios.hpp](/home/linfu/grad/me780/A2/planar_robot_2d/include/scenarios.hpp) and [src/scenarios.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/scenarios.cc).

Each scenario is described by a `ScenarioConfig` containing:

- Initial joint angles `initial_q`
- Initial joint velocities `initial_qdot`
- Initial desired joint angles `initial_q_d`
- Initial desired joint velocities `initial_qdot_d`
- Initial end-effector position `initial_xy`
- Initial desired end-effector position `initial_xy_d`
- Scenario duration `total_time_sec`
- A desired end-effector trajectory function

The factory function

```cpp
ScenarioConfig makeScenarioConfig(ScenarioMode mode, const RobotModel& robot);
```

is the single place where scenario-specific initialization is defined.

## Implemented Scenarios

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

When this scenario is active, the simulator also applies a wall contact force.

### External Disturbance

`runExternalDisturbance(double time_sec)` keeps the desired end-effector position fixed at:

```text
(1.0, 0.0)
```

When this scenario is active, the simulator applies a time-dependent external force disturbance.

## Control

The controller is implemented in [src/controller.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/controller.cc).

It uses joint-space PD control with gravity compensation:

```math
\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + G(q)
```

The commanded torque is then saturated elementwise by `tau_max`.

In `main.cc`, Cartesian references are converted to joint references with inverse kinematics, and desired joint velocity is approximated numerically from successive desired joint positions.

## Environment Interaction

The environment models are implemented in [src/simulator.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/simulator.cc).

### Wall Contact

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

The external disturbance is currently a downward Cartesian force applied between `t = 1.0 s` and `t = 1.5 s`:

```text
F = [0, -20]
```

This force is also converted into joint torques using `J(q)^T F`.

## Robot Model

The robot model is declared in [include/robot_model.hpp](/home/linfu/grad/me780/A2/planar_robot_2d/include/robot_model.hpp) and implemented in [src/robot_model.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/robot_model.cc).

It provides:

- Forward kinematics
- Inverse kinematics
- End-effector Jacobian
- Center-of-mass positions
- Mass matrix `M(q)`
- Coriolis vector `C(q, qdot)`
- Gravity vector `G(q)`
- Forward dynamics
- Total mechanical energy

The equations of motion follow:

```math
M(q)\ddot{q} + C(q,\dot{q}) + G(q) = \tau
```

and the forward dynamics are computed as:

```math
\ddot{q} = M(q)^{-1}\left(\tau - C(q,\dot{q}) - G(q)\right)
```

## Numerical Integration

The simulator advances the robot state with fourth-order Runge-Kutta integration in [src/simulator.cc](/home/linfu/grad/me780/A2/planar_robot_2d/src/simulator.cc).

`RobotState` stores:

- `q`: joint angles
- `qdot`: joint velocities
- `time`: current simulation time

`DesiredRobotState` stores:

- `q_d`: desired joint angles
- `qdot_d`: desired joint velocities
- `q_d_prev`: previous desired joint angles for numerical differentiation

## Logging

Each run creates a timestamped log file in `data/`.

The current logger records:

```text
time q1 q2 q1_d q2_d x y tau1 tau2 xy_d_x xy_d_y
```

This makes it easy to compare:

- actual vs desired joint motion
- actual vs desired end-effector position
- controller effort over time

## Notes On Inverse Kinematics

The current inverse kinematics implementation:

- Rejects targets outside the workspace
- Rejects targets with `x < 0`
- Chooses a solution with `q1` constrained to `[-pi/2, pi/2]`

So the solver is intentionally restricted and does not expose every possible elbow-up and elbow-down solution.

## License

This repository includes a `LICENSE` file at the project root.

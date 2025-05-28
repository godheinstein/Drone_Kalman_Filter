# Designing a Drone Simulation with Simplified Kalman Filter and Behavior Mechanics

---

## Project Overview

This project simulates a drone's flight behavior and implements a **simplified Kalman Filter** for robust state estimation. The core objective is to accurately estimate the drone's position and velocity using various sensor data, enabling precise navigation and control in a simulated environment.

## Project Objectives

* **Intelligent Drone Behavior:** Develop a comprehensive state machine that dictates the drone's actions, including automated takeoff, dynamic waypoint following, and controlled landing procedures.
* **Precise Drone Control:** Implement a custom controller to guide the drone smoothly along planned trajectories, maintaining a consistent yaw and adhering to defined speed limits.
* **Robust State Estimation:** Design and integrate a simplified Kalman Filter to fuse data from multiple sensors (IMU, sonar, GPS, magnetic sensor, barometer) for accurate real-time estimation of the drone's pose and twist.

## Key Implementations

### 1. Behavior Node: Autonomous State Machine

The behavior node orchestrates the drone's high-level actions through a **finite state machine**. This enables the drone to transition seamlessly between critical operational phases:

* **`TAKEOFF`**: Ascends to a predefined cruise height.
* **`INITIAL`**: Navigates to a starting air position after takeoff.
* **`TURTLE_POSITION`**: Tracks and follows a moving Turtlebot.
* **`TURTLE_WAYPOINT`**: Moves to the Turtlebot's current target destination.
* **`LANDING`**: Initiates a controlled descent to the initial ground position.
* **`END`**: Final state after a successful landing.

The state machine ensures the drone systematically completes a three-step cycle (move to Turtlebot, move to Turtlebot's waypoint, return to initial air position) and lands only after the Turtlebot has ceased movement and the cycle is complete.

### 2. Controller Node: Holonomic Pure Pursuit

The controller guides the drone along a given path using a holonomic pure pursuit approach. Unlike traditional pure pursuit, this variant allows the drone to move in any direction (not limited by curvature), making it suitable for omnidirectional drone movement. It determines the necessary linear velocities in the X, Y, and Z axes to reach a **lookahead point** on the path, while also maintaining a constant yaw velocity.

### 3. Estimator Node: Simplified Kalman Filter

The core of the estimation system is a **simplified Kalman Filter**, which fuses noisy sensor data to produce accurate state estimates (position and velocity in X, Y, Z, and yaw and yaw velocity). The filter operates in two main stages: Prediction and Correction.

#### 3.1. Prediction Stage

The prediction stage uses the drone's motion model and IMU data to estimate the next state and its uncertainty. This is governed by the following equations:

* **State Prediction:**
```math
\begin{align}
\mathbf{\hat{X}}_{k|k-1} &= f(\mathbf{\hat{X}}_{k|k-1}, \mathbf{U}_k) \\
&\approx \mathbf{F}_k \mathbf{\hat{X}}_{k-1|k-1} + \mathbf{W}_k \mathbf{U}_k
\end{align}
```
Where $\mathbf{\hat{X}}$ is the state vector (position, velocity, yaw, yaw rate), $\mathbf{U}$ is the control input (IMU linear accelerations and angular velocity), $\mathbf{F}$ is the state transition matrix, and $\mathbf{W}$ is the control input matrix. For example, for the $x$-axis, the prediction is:
    
```math
\begin{align}
\begin{bmatrix} x_{k|k-1} \\ \dot{x}_{k|k-1} \end{bmatrix} &=
\begin{bmatrix} x_{k-1|k-1} + \dot{x}_{k-1|k-1} \Delta t + \frac{1}{2}(\Delta t)^2 a_{x,k} \\
\dot{x}_{k-1|k-1} + a_{x,k}\Delta t
\end{bmatrix}
\end{align}
```
Here, $a_{x,k}$ is the acceleration in the world frame, derived from IMU measurements and the drone's yaw:
```math
\begin{align}
\begin{bmatrix} a_{x,k} \\ a_{y,k} \end{bmatrix} &=
\begin{bmatrix} \cos{\psi} & -\sin{\psi} \\ \sin{\psi} & \cos{\psi} \end{bmatrix}
\begin{bmatrix} u_{x,k} \\ u_{y,k} \end{bmatrix}
\end{align}
```

* **Covariance Prediction:**
```math
\begin{align}
\mathbf{P}_{k|k-1} &= \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^\top + \mathbf{W}_k \mathbf{Q}_k \mathbf{W}_k^\top
\end{align}
```
Where $\mathbf{P}$ is the covariance matrix representing uncertainty, and $\mathbf{Q}$ is the process noise covariance matrix (e.g., $\mathbf{Q}_x = \begin{bmatrix} \sigma_{imu,x}^2 & 0 \\ 0 & \sigma_{imu,y}^2 \end{bmatrix}$).

#### 3.2. Correction Stage

When new sensor measurements become available, the correction stage updates the state estimate and covariance, minimizing the error between the prediction and the measurement.

* **Kalman Gain:**
```math
\begin{align}
    \mathbf{K}_k &= \mathbf{P}_{k|k-1} \mathbf{H}_k^\top
    \left(
    \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^\top
    + \mathbf{V}_k \mathbf{R}_k \mathbf{V}_k^\top
    \right)^{-1}
\end{align}
```
* **State Update:**
```math
\begin{align}
    \mathbf{\hat{X}}_{k|k} &= \mathbf{\hat{X}}_{k|k-1} + \mathbf{K}_k
    \left(
    \mathbf{Y}_k - \mathbf{H}_k \mathbf{\hat{X}}_{k|k-1}
    \right)
\end{align}
```
* **Covariance Update:**
```math
\begin{align}
    \mathbf{P}_{k|k} &= \mathbf{P}_{k|k-1} - \mathbf{K}_k \mathbf{H}_k \mathbf{P}_{k|k-1}
\end{align}
```
Here, $\mathbf{Y}$ is the measurement, $\mathbf{H}$ is the measurement matrix (Jacobian of the measurement model), $\mathbf{V}$ is the measurement frame transformation, and $\mathbf{R}$ is the measurement noise covariance.

**Sensor-Specific Corrections:**

* **Sonar:** Provides direct height measurements along the $z$-axis.
```math
\mathbf{Y}_{snr,z,k} = z_{snr} = z_{k|k-1}+ \varepsilon_{snr,k}, \quad \mathbf{H}_{snr,z,k} = \begin{bmatrix}1 & 0 \end{bmatrix}, \quad \mathbf{R}_{snr,z,k} = \sigma^2_{snr,z}
```
* **GPS:** Provides 3D position measurements in latitude, longitude, and altitude, which are converted to ECEF (Earth-Centered, Earth-Fixed) and then to the local world frame.
**ECEF Conversion:**
```math
\begin{align}
    e^2 &= 1 - \frac{b^2}{a^2} \\
    N(\varphi) &= \frac{a}{\sqrt{1 - e^2 \sin^2(\varphi)}} \\
    \begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} &=
    \begin{bmatrix}
    (N(\varphi) + h) \cos(\varphi) \cos(\lambda) \\
    (N(\varphi) + h) \cos(\varphi) \sin(\lambda) \\
    \left( \frac{b^2}{a^2} N(\varphi) + h \right) \sin(\varphi)
    \end{bmatrix}
\end{align}
```
**NED to World Frame Transformation:**
```math
\begin{align}
    \mathbf{R}_{m/n} &= \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix} \\
    \begin{bmatrix} x_{gps} \\ y_{gps} \\ z_{gps} \end{bmatrix} &= \mathbf{R}_{m/n}
    \begin{bmatrix} x_n \\ y_n \\ z_n \end{bmatrix} +
    \begin{bmatrix} x_0 \\ y_0 \\ z_0 \end{bmatrix}
\end{align}
```
The Jacobian $\mathbf{H}$ for each axis ($x, y, z$) from GPS is $\begin{bmatrix}1 & 0 \end{bmatrix}$, with corresponding variances $\sigma^2_{gps,x}$, $\sigma^2_{gps,y}$, $\sigma^2_{gps,z}$.
* **Magnetic Sensor & Barometer:** These sensors would also provide corrections for specific states (e.g., yaw for magnetic, altitude for barometer), similar to the sonar.

## Software Requirements
1. Ubuntu 22.04 with ROS2 Humble

## Installation and Usage Instructions
1. In Ubuntu, open a terminal with `Ctrl+Alt+T`.

2. On the terminal, clone the GitHub repository:
    ```bash
    mkdir ros2_ws
    cd ~/ros2_ws
    git clone https://github.com/godheinstein/Drone_Kalman_Filter.git
    ```
3. Let the `ros2_ws` folder be known as the **workspace** folder. Navigate into the workspace folder, and build the files.
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```
    Wait for about a minute.

## Running the Drone Simulation

To run the drone simulation:

1.  **Build the Workspace:** Ensure your `ros2_ws` workspace is built.
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```
2.  **Source the Setup:**
    ```bash
    source install/setup.bash
    ```
3.  **Launch the Simulation:**
    ```bash
    ros2 launch ee4308_bringup proj2.launch.py
    ```

**Configuration (`proj2.yaml` parameters):**

* `use_ground_truth`: Set to `false` for real estimation.
* `reached_thres_`: Waypoint reaching threshold (e.g., `0.3` m).
* `cruise_height_`: Target cruising altitude (e.g., `5.0` m).
* `max_xy_vel_`: Maximum horizontal velocity (e.g., `1.0` m/s).
* `max_z_vel_`: Maximum vertical velocity (e.g., `0.5` m/s).
* `yaw_vel_`: Desired constant yaw velocity (e.g., `-0.3` rad/s).
* `lookahead_distance_`: Lookahead distance for the controller (e.g., `1.0` m).
* `verbose`: Set to `true` to enable terminal output for debugging and demonstration.

---

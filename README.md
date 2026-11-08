# Autonomous Robotic Control System

**Jetson Nano | ROS | Python | C++ | LiDAR**

Developed autonomous navigation and control algorithms for a Jetson Nano-based robotic vehicle using ROS, LiDAR, and IMU sensors. Implemented wall-following control, obstacle avoidance, and steering and speed regulation. Separately, developed a 2D occupancy grid mapping algorithm using LiDAR and wheel odometry to represent the surrounding environment.

<p align="center">
  <img src="https://github.com/user-attachments/assets/b5b6473e-17d9-47ed-9970-80759eb92216" alt="MacAEV Robotic Vehicle" width="650">
</p>

## Project Demonstration

<p align="center">
  <img src="https://github.com/user-attachments/assets/c15bcdfb-f20f-4e93-a787-afc3cae70a14" alt="Autonomous Robotic System Demonstration" width="330">
</p>

## Key Features

### Autonomous Navigation & Wall Following

- Implemented a **PD feedback controller** to calculate steering corrections based on the vehicle's distance from surrounding walls.
- Processed LiDAR measurements to estimate the vehicle's position relative to nearby walls and maintain a centered path.
- Adjusted steering and speed commands to guide the vehicle through its environment.

### Obstacle Detection & Avoidance

- Used LiDAR distance measurements to identify obstacles in the vehicle's path.
- Implemented distance-based speed adjustments to slow the vehicle as it approaches nearby obstacles.
- Explored **virtual barrier-based navigation** using LiDAR measurements and quadratic programming.

### Occupancy Grid Mapping

Developed a separate mapping algorithm to represent the surrounding environment using sensor measurements.

- Generated a **2D occupancy grid map** using LiDAR and wheel odometry.
- Applied probabilistic log-odds updates to classify map cells as **free, occupied, or unknown**.
- Updated the occupancy grid as new sensor measurements became available.

### Sensor Integration & Odometry

- Used **ROS publishers and subscribers** to exchange LiDAR, IMU, and odometry data between system components.
- Estimated the vehicle's position and heading using wheel speed measurements and IMU orientation data.
- Published odometry and sensor information for navigation and mapping.

## Repository Structure

| File | Description |
|---|---|
| `navigation.py` | LiDAR-based wall following, PD steering control, and speed adjustments |
| `occupancygridmap.py` | Probabilistic 2D occupancy grid mapping using LiDAR and odometry |
| `virtual_barrier.py` | Experimental virtual barrier navigation using quadratic programming |
| `experiment.cpp` | ROS vehicle interface, odometry processing, and motor control |
| `params.yaml` | Vehicle configuration, sensor settings, ROS topics, and controller parameters |

## Technologies Used

`Python` · `C++` · `ROS` · `Jetson Nano` · `LiDAR` · `IMU` · `VESC` · `PD Control` · `Occupancy Grid Mapping`

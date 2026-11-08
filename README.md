# Autonomous Robotic Control System

**Jetson Nano | ROS | Python | C++ | LiDAR**

Developed autonomous navigation and control algorithms for a Jetson Nano-based robotic vehicle using ROS, LiDAR, and IMU sensors. Implemented wall-following control, obstacle avoidance, and steering and speed regulation. Separately, developed a 2D occupancy grid mapping algorithm using LiDAR and wheel odometry to represent the surrounding environment.

<p align="center">
  <img src="https://github.com/user-attachments/assets/b5b6473e-17d9-47ed-9970-80759eb92216" alt="MacAEV Autonomous Robotic Vehicle" width="650">
</p>

## Key Features

### Wall Following & Vehicle Control
- Implemented a **PD controller** to adjust steering based on the vehicle's distance from surrounding walls.
- Used LiDAR measurements to estimate distances and maintain the vehicle's position between walls.
- Adjusted steering and speed commands based on the surrounding environment.

### Occupancy Grid Mapping
- Developed a **2D occupancy grid map** using LiDAR and wheel odometry.
- Used probabilistic updates to classify areas as **free, occupied, or unknown**.
- Updated the map as new LiDAR measurements were received.

### Sensor Integration & Odometry
- Used **ROS publishers and subscribers** to exchange LiDAR, IMU, and wheel odometry data.
- Estimated vehicle position using wheel speed and IMU heading measurements.
- Published odometry information for use by other ROS nodes.

### Obstacle Detection & Navigation
- Used LiDAR measurements to detect obstacles ahead of the vehicle.
- Adjusted vehicle speed based on the distance to nearby obstacles.
- Explored virtual barrier navigation using LiDAR measurements and quadratic programming.

## Repository Structure

| File | Description |
|---|---|
| `navigation.py` | LiDAR-based wall following, PD steering control, and speed adjustments |
| `occupancygridmap.py` | 2D occupancy grid mapping using LiDAR and odometry |
| `virtual_barrier.py` | Experimental virtual barrier navigation using quadratic programming |
| `experiment.cpp` | ROS vehicle interface, odometry processing, and motor control |
| `params.yaml` | Vehicle configuration, sensor settings, ROS topics, and controller parameters |

## Technologies Used

`Python` · `C++` · `ROS` · `Jetson Nano` · `LiDAR` · `IMU` · `VESC` · `PD Control` · `Occupancy Grid Mapping`

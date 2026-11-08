</p># Autonomous Robotic Control System

**Jetson Nano | ROS | Python | C++ | LiDAR**

Developed an autonomous robotic vehicle using ROS, LiDAR, and odometry for navigation and obstacle detection. The project involved creating control algorithms to help the vehicle follow walls, adjust its steering and speed, and build a map of its surroundings using sensor measurements.

## Key Features

### 1. Wall Following & Vehicle Control
- Implemented a **PD controller** to adjust the vehicle's steering based on its distance from surrounding walls.
- Used LiDAR measurements to estimate the vehicle's position relative to the left and right walls.
- Adjusted the steering angle and vehicle speed to maintain a safe path while navigating.

### 2. Occupancy Grid Mapping
- Developed a **2D occupancy grid map** using LiDAR and wheel odometry measurements.
- Used a probability-based approach to classify areas as **free, occupied, or unknown**.
- Updated the map as the vehicle moved and received new sensor measurements.

### 3. Sensor Integration & Odometry
- Used **ROS publishers and subscribers** to process LiDAR, IMU, and wheel odometry data.
- Estimated the vehicle's position using wheel speed measurements and IMU heading information.
- Published updated odometry and sensor data for use by the navigation system.

### 4. Obstacle Detection & Navigation
- Used LiDAR distance measurements to identify obstacles in the vehicle's path.
- Implemented speed adjustments based on the distance to nearby obstacles.
- Explored virtual barrier-based navigation using LiDAR measurements and optimization methods.

  <p align="center">
  <img 
    src="https://github.com/user-attachments/assets/b5b6473e-17d9-47ed-9970-80759eb92216"
    alt="MacAEV Autonomous Robotic Vehicle"
    width="650"
  />

## Repository Structure

| File | Description |
|---|---|
| `navigation.py` | Implements LiDAR-based wall following, PD steering control, and speed adjustments. |
| `occupancygridmap.py` | Creates and updates a 2D occupancy grid using LiDAR and odometry data. |
| `virtual_barrier.py` | Experimental virtual barrier controller using LiDAR measurements and quadratic programming. |
| `experiment.cpp` | Handles the ROS vehicle interface, including sensor data, odometry, and steering and motor commands. |
| `params.yaml` | Contains vehicle parameters, sensor settings, ROS topics, and controller gains. |

## Technologies Used

`Python` · `C++` · `ROS` · `Jetson Nano` · `LiDAR` · `IMU` · `PD Control` · `Occupancy Grid Mapping`

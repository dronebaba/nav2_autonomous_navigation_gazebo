# nav2_autonomous_gazebo

## Overview
This repository contains a ROS 2 package for autonomous navigation using Nav2 in Gazebo. It is developed for ROS 2 Humble and designed to simulate autonomous robots navigating in a Gazebo environment.

## Features
- Autonomous navigation using Nav2
- Integration with Gazebo for simulation
- Custom URDF robot model
- Path planning and obstacle avoidance
- Compatible with ROS 2 Humble

## Installation
### Prerequisites
- ROS 2 Humble installed ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- Gazebo installed ([Gazebo Installation](https://gazebosim.org/docs/latest/ros_installation/))

### Clone the Repository
```sh
cd ~/ros2_ws/src
git clone https://github.com/dronebaba/nav2_autonomous_navigation_gazebo.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select nav2_autonomous_gazebo
source install/setup.bash
```

## Building the Selected Package
To build only the `nav2_autonomous_gazebo` package, use the following commands:

### Navigate to Your ROS 2 Workspace
```sh
cd ~/ros2_ws
```

### Build the Selected Package
```sh
colcon build --packages-select nav2_autonomous_gazebo
```

### Source the Setup File
```sh
source install/setup.bash
```

### Verify the Build
```sh
ros2 pkg list | grep nav2_autonomous_gazebo
```

## Usage
### Launch Gazebo and RViz2 Simulation
```sh
ros2 launch nav2_autonomous_gazebo sim_nav.launch.py
```
### Set a Specific Navigation Goal
To move the robot to a specific (x, y) position and yaw orientation, use the following command. You can change the x, y, and yaw values as needed:
```sh
ros2 run nav2_autonomous_gazebo set_nav_goal x=3.0 y=2.0 yaw=1.57
```

### Implement Multi-Goal Navigation
To enable the robot to autonomously visit multiple locations, use the following command:
```sh
ros2 launch nav2_autonomous_gazebo set_wp_client
```
The current configuration includes three waypoints. You can add more waypoints as needed. To do so, use the following command in `set_wp_client.cpp` file in the `src/` folder:
```sh
create_pose(x, y, yaw)
```
For example:
```sh
create_pose(6.6, 6.6, 0)
```
### Keyboard Teleportation for Mapping
To create a map by manually moving the robot in the environment using keyboard commands, use one of the following commands:
```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_gz
```
### Check Coordinates in the Map
To check a specific point's coordinates on the map, run the following command:
```sh
ros2 topic echo /clicked_point
```
Then, in RViz, select the 'Publish Point' tool and click anywhere on the map to see the coordinates.
This will give the following output:
```sh
header:
  stamp:
    sec: 443
    nanosec: 58000000
  frame_id: map
point:
  x: 2.877362012863159
  y: 4.212574005126953
  z: -0.001373291015625
```
### Terminate or Cancel the Simulation
To stop Gazebo and RViz, use one of the following commands:
```sh
- Press (ctrl + c)
- Press (ctrl + \)
```

## Configuration
If you want to tune the parameters, open the `nav2_params.yaml` file in the `config/` folder. This file contains all the Nav2 parameters.

For parameter tuning, follow these links:
- [Configuring DWB Controller]((https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html))
- [Configuring Velocity Smoother]((https://docs.nav2.org/configuration/packages/configuring-velocity-smoother.html))
- [Configuring AMCL]((https://docs.nav2.org/configuration/packages/configuring-amcl.html))

## Short Explanation
- How you approached localization, planning, and obstacle avoidance.
  - Localization -
    Localization consists of two main approaches:
    - SLAM (Simultaneous Localization and Mapping) – In this approach, the robot moves through an unknown environment while simultaneously building a map of it. SLAM (slam_toolbox) is used when the environment is unexplored, and we need to generate a new map. The generated map can then be utilized for navigation and localization. A map was recorded for the custom created world, using `teleop_twist_keyboard`. This process can also be carried out autonomously using nav2 while performing SLAM. `slam_toolbox` can also be used for just localization by setting the mode explicitly in the parameters file. 

    - AMCL (Adaptive Monte Carlo Localization) – AMCL is a probabilistic localization algorithm that helps a robot determine its position within a known map. A map was recorded using `slam_toolbox` and saved within the package, in yaml and pgm format. This map was provided to AMCL using `map_server`, and an initial pose estimate was provided using a ros2 cpp node, as required in the assignment. 
  
  - Navigation 
    Navigation consists of two main approaches:
    - Local Navigation – This approach focuses on obstacle avoidance and real-time path adjustments. I used the `DWB (Dynamic Window Approach)` controller as the default local planner. The robot utilizes a Laser Scanner (RPLidar) to continuously scan its surroundings while moving toward the goal. The scanned data helps the local planner detect obstacles and adjust its trajectory to prevent collisions.

    - Global Navigation – This approach is responsible for generating an optimal path from the start position to the goal using a pre-existing map. I used the `Navfn Planner plugin`, which calculates a global path based on the map generated by SLAM. The robot follows this path while relying on the local planner for dynamic obstacle avoidance which takes into account the real-time LiDAR scan data.
  
  - Obstacle Avoidance
    - I used a Laser Scanner (RPLidar simulated in GzSim Harmonic) to help the robot detect obstacles in real-time. The RPLidar is responsible for both mapping the environment and providing real-time obstacle detection while the robot moves toward its goal. If an obstacle is detected, the robot dynamically adjusts its path to avoid collisions. The continuous scanning ensures that the robot receives updated data, enabling it to navigate safely through the environment, using the local planner provided by nav2. 

- What challenges you faced and how you solved them.
  While working on the project, I encountered two main challenges:

  - Adding Moving Obstacles and Modifying the Environment :
  I needed to introduce moving obstacles and dynamically modify the environment as per the project requirements. To achieve this, I referred to the official GazeboSim documentation and used the Actor feature (tag) to add moving elements within a specific area and enabled the movement using the script tag.

  - Tuning the Robot’s Trajectory and Velocity:
   Since trajectory and velocity tuning were new to me, I had to carefully adjust various parameters for optimal performance. I relied on the official Nav2 documentation (tuning guide), studied all relevant parameters, and used them to fine-tune the robot’s movement for smooth and efficient navigation.







## Contributing
Contributions are welcome! Please create a pull request or open an issue.

## License
This project is licensed under the MIT License.

## Contact
For any questions, contact [ajaykrucheniya@gmail.com](mailto:ajaykrucheniya@gmail.com).
# Autonomous Navigation, SLAM, and Localization

This project explored a complete mobile-robot navigation pipeline for a TurtleBot3 in a maze-like environment. The robot used onboard sensing and odometry to build an occupancy-grid map of an unknown space, then autonomously explored the environment, planned collision-free paths, and navigated to requested goals. The system was developed in ROS and demonstrated in Gazebo and RViz.

The implementation combines several classic robotics components: SLAM-based mapping, frontier-based exploration, configuration-space path planning, and AMCL-based localization. The repository includes separate ROS nodes for motion control, path planning, frontier selection, and localization, along with launch files for both simulation and hardware-oriented workflows.

## Demo

<video controls width="100%">
  <source src="docs/media/slam-navigation-demo.mp4" type="video/mp4">
</video>

The clip shows the robot navigating in simulation while the map, planned path, and exploration behavior are visualized in RViz.

## Project Objectives

- Build an occupancy-grid map of an unknown environment.
- Identify unexplored regions using frontier-based exploration.
- Select navigation goals for exploration and return-to-home behavior.
- Plan safe routes with A* using a configuration-space representation.
- Follow the planned paths while avoiding nearby obstacles.
- Localize the robot on a known map with AMCL and navigate to a requested goal.

## System Overview

LiDAR and odometry
        ↓
SLAM and occupancy-grid mapping
        ↓
Frontier detection
        ↓
Configuration-space processing
        ↓
A* path planning
        ↓
Path following and robot control

Saved map and sensor data
        ↓
AMCL localization
        ↓
Pose estimate
        ↓
Goal planning and navigation

## Main Components

### Mapping and SLAM
The project uses LiDAR and odometry data to build an occupancy grid of the environment. The repository includes gmapping-style SLAM configuration and map files for the maze environment, which are used by the exploration and localization workflow.

### Configuration Space
The planner expands obstacles into a safety margin so the robot can avoid physically impossible paths. This configuration-space inflation is used to make the generated routes more robust when the robot is following them.

### Frontier Exploration
Frontier-based exploration identifies the boundaries between known and unknown space. The frontier node evaluates these regions and selects promising targets for the robot to explore next.

### A* Path Planning
Once a goal is selected, the planner computes a collision-free route through the map. The implementation uses A* over the occupancy grid and incorporates configuration-space information to avoid narrow or unsafe passages.

### Path Following
A local controller sends velocity commands to the robot as it follows the planned path. The repository includes nodes for trajectory tracking and obstacle-aware motion behavior.

### AMCL Localization
After a map has been created, AMCL estimates the robot’s pose on that map from sensor data and motion estimates. This allows the system to switch from exploration to goal-directed navigation with a more reliable pose estimate.

## Simulation and Visualization

Gazebo simulates the TurtleBot3 and maze environment, while RViz displays the generated map, laser observations, target goals, path planning output, and localization state. The repository also includes launch files for both simulation and hardware-oriented workflows.

## Repository Structure

```text
.
├── docs/
│   └── media/
├── lab2/
├── lab3/
├── lab4/
├── std_msg/
├── .gitignore
└── README.md
```

## Technologies

- ROS
- Python
- TurtleBot3
- Gazebo
- RViz
- GMapping-style SLAM configuration
- AMCL
- A* path planning
- Frontier-based exploration

## Team and Course Context

This project was developed as a team effort for RBE 3002 – Unified Robotics IV at Worcester Polytechnic Institute. The work was originally organized under RBE300X-Lab/RBE3002_B24_Team02.
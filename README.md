# Robotics SLAM Workshop

This project is a hands-on workshop from the Robotics course of Leiden University, which focused on Simultaneous Localisation and Mapping (SLAM) using a two-wheeled robot equipped with a LiDAR sensor. The goal is to develop Python code that enables the robot to autonomously navigate and avoid obstacles in a simulated multi-room environment, ultimately reaching a designated target area.

## Key Features
- Use of CoppeliaSim EDU 4.9.0 simulation software to emulate the PioneerP3DX robot and its SickTIM310 LiDAR sensor.
- Processing LiDAR data to design robust navigation and obstacle avoidance strategies.
- Introduction to core SLAM challenges, including localisation and mapping.
- Modular Python programming to automate multi-room traversal tasks.

## Installation & Setup

### Install CoppeliaSim
1. Download and install [CoppeliaSim EDU 4.9.0 rev 6](https://www.coppeliarobotics.com/downloads) for your OS.

### Install dependencies
```
python -m pip install numpy opencv-python matplotlib coppeliasim-zmqremoteapi-client cbor2 keyboard
```

## Running the Simulation
1. Launch CoppeliaSim and open the scene file: `scenes/room_static1.ttt`.
2. Run the main Python script to control the robot:
```

python run.py

```




Contributions and feedback are welcome to enhance the robot navigation and SLAM strategies!


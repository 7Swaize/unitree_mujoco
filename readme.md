# Introduction
## Unitree mujoco
This is a simulator developed based on `Unitree SDK2` and `Mujoco`. This is **FORK** of the original and much has changed. 

This simulator is intended to be used with the [Go2-Control Wrapper](https://github.com/7Swaize/go2-control.git). **ALL** dependency information, installations, and API Documentation are listed there.

Joystick functionality is technically preserved.

## Directory Structure
- `iceoryx_interfaces/`: Shared Iceoryx2 client-server configurations needed for IPC
- `simulate/resources/`: Resources used to load the robot model and scene
- `simulate/src/cpp/`: Core Mujoco Simulation written in C++
- `simulate/src/python/`: Python bridge between the `go2-control` wrapper and the C++ simulation

## Internally Supported Unitree SDK2 Messages:
**Current version only supports low-level development, mainly used for sim to real verification of controller**
- `LowCmd`: Motor control commands
- `LowState`: Motor state information
- `SportModeState`: Robot position and velocity data
- `IMUState`: Torso IMU state at `rt/secondary_imu` topic (G1 only)

Note:
1. The numbering of the motors corresponds to the actual robot hardware. Specific details can be found in the [Unitree documentation](https://support.unitree.com/home/zh/developer).
2. In the actual robot hardware, the `SportModeState` message is not readable after the built-in motion control service is turned off. However, the simulator retains this message to allow users to utilize the position and velocity information for analyzing the developed control programs.

## Message (DDS IDL) Type Description
- Unitree Go2, B2, H1, B2w, Go2w robots use unitree_go idl for low-level communication.
- Unitree G1, H1-2 robot uses unitree_hg idl for low-level communication.


# Configuration
## C++ Simulator
The configuration file for the C++ simulator is located at `simulate/resources/config/`:

- `camera.yaml`: Configuration file for Go2 simulator camera
- `global.yaml`: Configuration file for the C++ simulation


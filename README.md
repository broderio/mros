# MROS

This library provides a publisher/subscriber inter-process communication (IPC) mechanism. It includes message types, memory management, socket communication, and core publisher/subscriber functionality.

## Directory Structure

- `include`: Contains all the header files.
  - `jrdf`: Contains classes for JSON parsing and generating a robot model.
  - `linalg`: Contains classes for linear algebra operations, such as `matrix`, `quaternion`, and `vector`.
  - `messages`: Defines the message types used in the library.
    - `geometry_msgs`: Contains geometric message types like `point`, `pose`, `quaternion`, `transform`, `twist`, and `vector3`.
    - `mbot_msgs`: Contains specific message types like `lidarScan`, `mbotEncoders`, `pose2d`, and `twist2d`.
    - `sensor_msgs`: Contains sensor message types like `jointState`.
    - `std_msgs`: Contains standard message types like `header`, `string`, and `time`.
  - `mros`: Contains the core functionality of the publisher/subscriber mechanism.
  - `smem`: Contains shared memory management classes.
  - `socket`: Contains socket communication classes for both TCP and UDP protocols.
- `robots`: Contains JSON files for robot models.
- `src`: Contains the source files corresponding to the header files in `include`.
- `test`: Contains unit tests for the different components of the library.

## Building

This project uses CMake. To build the project, navigate to the project root and run:

```bash
mkdir build
cd build
cmake ..
make
```
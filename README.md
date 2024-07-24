# MROS

This library provides a publisher/subscriber inter-process communication (IPC) mechanism. It includes message types, memory management, socket communication, and core publisher/subscriber functionality.

## Libraries

- `include`: Contains the header files for the MROS libraries
  - `mros`: Contains the publisher/subscriber system.
    - `core`: Contains the core functionality of MROS (`node`, `publisher`, `subscriber`, `mediator`, and `service`).
    - `utils`: Contains utility classes such for command line argument parsing, callback functions, logging, and signal handling.
    - `private_msgs`: Contains the private message definitions used for establishing connections between nodes.
  - `messages`: Defines the message types used in the library.
    - `geometry_msgs`: Contains a subset of the [ROS geometry_msgs](https://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html).
    - `mbot_msgs`: Contains message types specific to the MBot platform`.
    - `sensor_msgs`: Contains a subset of the [ROS sensor_msgs](https://docs.ros.org/en/noetic/api/sensor_msgs/html/index-msg.html).
    - `std_msgs`: Contains a subset of the [ROS std_msgs](https://docs.ros.org/en/melodic/api/std_msgs/html/index-msg.html).
  - `linalg`: Contains classes for linear algebra operations, such as `matrix`, `quaternion`, `rotation`, and `vector`.
  - `kineval`: Contains classes for kinematic evalutation (forward/inverse kinematic solvers, robot joint/link representations, and kinematic trees)
  - `smem`: Contains shared memory management classes.
  - `socket`: Contains socket communication classes for both TCP and UDP protocols.
- `robots`: Contains JSON Robot Description Files (JRDF) for robot models.
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

To install the MROS libraries locally, run:
```bash
sudo make install
```
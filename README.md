# MROS

This library provides a publisher/subscriber inter-process communication (IPC) mechanism. It includes message types, memory management, socket communication, and core publisher/subscriber functionality.

## Directory Structure

- `include`: Contains all the header files.
  - `messages`: Defines the message types used in the library.
    - `msg_types`: Contains specific message types like `lidarScan`, `mbotEncoders`, `pose2d`, and `twist2d`.
  - `mros`: Contains the core functionality of the publisher/subscriber mechanism.
  - `smem`: Contains shared memory management classes.
  - `socket`: Contains socket communication classes.
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
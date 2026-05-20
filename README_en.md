# BPX SDK Open

`bpx_sdk_open` provides a lightweight C++ SDK for reading BPX robot state and sending motion-level or joint-level control commands.

Current version: `1.0.1`

The SDK offers three usage modes:

| Mode | Description |
| --- | --- |
| State Query Layer | Subscribes to and reads robot state only; does not send control commands. Suitable for telemetry and state-monitoring use cases. |
| Motion Control Layer | Sends high-level motion commands such as stand up, sit down, damping, and velocity control, while also being able to read robot state. |
| Joint Control Layer | Directly sends 12-DOF joint targets and torque feed-forward commands, while being able to read both normal state and high-rate joint state. |

The current SDK interface and state fields are an initial implementation; additional interfaces and readable robot state fields will be added incrementally in future releases.

## Directory Structure

```text
bpx_sdk_open/
  CMakeLists.txt
  include/
    bpx_sdk_config.h
    bpx_sdk_version.h
    motion_types.h
    request_robot_state.h
    motion_level_control.h
    joint_level_control.h
  lib/
    libbpx_sdk_x86_64.so
    libbpx_sdk_aarch64.so
  example/
    request_robot_state_example.cpp
    motion_level_control_example.cpp
    joint_level_control_example.cpp
```

## Common Configuration

Public configuration constants are defined in `include/bpx_sdk_config.h`.

| Name | Description |
| --- | --- |
| `DEFAULT_SERVER_IP` | Default robot IP used by the SDK. |
| `DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT` | Local port for receiving robot state packets. |
| `DEFAULT_CLIENT_JOINT_STATE_UDP_PORT` | Local port reserved for receiving joint state upload data. |

The command destination port is fixed on the robot side and is not exposed as a public SDK configuration.

The robot wireless IP is `192.168.0.1`; the wired IP is `10.21.20.1`. The SDK defaults to `10.21.20.1`. To change the default IP, either modify `include/bpx_sdk_config.h` directly, or call `setRobotIp` in your program to set the target robot IP at runtime.

## Motion State and Gait Types

Header: `include/motion_types.h`

The SDK provides type-safe enumerations for motion state and gait, which can be used to read the current and last motion state and gait.

Motion state `bpx_sdk::MotionState`:

| Enum Value | Raw Value | Description |
| --- | --- | --- |
| `LyingDown` | `0` | Lying down. |
| `StandingUp` | `1` | Standing up. |
| `Passive` | `2` | Passive (damping) state. |
| `SitDown` | `3` | Sitting down. |
| `Motion` | `6` | Active motion state. |

Gait `bpx_sdk::MotionGait`:

| Enum Value | Raw Value |
| --- | --- |
| `Walk` | `0` |
| `Bipedal` | `3` |
| `Flip` | `4` |
| `WalkPhase` | `6` |
| `PoseTracking` | `7` |
| `Running` | `8` |
| `WalkPeriod` | `10` |

## State Query Layer

Header: `include/request_robot_state.h`

Class: `bpx_sdk::RequestRobotState`

This layer subscribes to robot state data and provides read-only query interfaces. Use this layer when the program only needs telemetry data and does not need to send control commands.

The joint data and IMU data obtained through the state query layer arrive at a lower rate, making it suitable for state monitoring and low-frequency telemetry. If high-rate joint state is required, use the `HighRate` interfaces provided by the joint control layer.

Connection and setup:

```cpp
bpx_sdk::RequestRobotState robot_state;
robot_state.setRobotIp(bpx_sdk::DEFAULT_SERVER_IP);
robot_state.setRobotStateUploadPort(bpx_sdk::DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
robot_state.setTcpLocalPort(0);
robot_state.setRobotStateUploadRate(100);

if (!robot_state.connect()) {
    return 1;
}
```

Main state interfaces:

| Interface | Description |
| --- | --- |
| `getJointPosition(float[12])` | Joint positions in radians. |
| `getJointVelocity(float[12])` | Joint velocities in radians/second. |
| `getJointTorque(float[12])` | Joint torques. |
| `getImuRpy(float[3])` | Body roll, pitch, and yaw angles. |
| `getImuQuat(float[4])` | Body quaternion. |
| `getImuAcc(float[3])` | IMU linear acceleration. |
| `getImuOmega(float[3])` | IMU angular velocity. |
| `getCurrentVelocityBody(float[3])` | Current velocity in the body frame. |
| `getLegOdom(float[3])` | Leg odometry data. |
| `getCurrentMotionState(uint8_t*)` | Current motion state (raw value). |
| `getCurrentGait(uint8_t*)` | Current gait (raw value). |
| `getLastMotionState(uint8_t*)` | Last motion state (raw value). |
| `getLastGait(uint8_t*)` | Last gait (raw value). |
| `getCurrentMotionState(MotionState*)` | Current motion state as enum. |
| `getCurrentGait(MotionGait*)` | Current gait as enum. |
| `getLastMotionState(MotionState*)` | Last motion state as enum. |
| `getLastGait(MotionGait*)` | Last gait as enum. |
| `getMaxVelocity(float[3])` | Current maximum velocity limits. |
| `getBatteryLevel(uint8_t*)` | Battery level as a percentage. |
| `getBatteryCurrent(float*)` | Battery current. |
| `getMotorTemperature(float[12])` | Motor temperatures. |
| `getDriverTemperature(float[12])` | Driver temperatures. |
| `getJointStateTimestamp(uint32_t*)` | Timestamp of the latest joint state packet. |
| `getImuTimestamp(uint32_t*)` | Timestamp of the latest IMU data packet. |
| `getOdometryTimestamp(uint32_t*)` | Timestamp of the latest odometry packet. |
| `getMotionStateTimestamp(uint32_t*)` | Timestamp of the latest motion state packet. |
| `getBatteryTimestamp(uint32_t*)` | Timestamp of the latest battery state packet. |

The SDK also provides optional array-returning helper interfaces such as `getJointPositionArray()`, `getImuRpyArray()`, and `getBatteryLevelValue()`. These return `std::optional`, allowing callers to check success more concisely. Motion state and gait equivalents—`getCurrentMotionStateEnum()`, `getCurrentGaitEnum()`, `getLastMotionStateEnum()`, and `getLastGaitEnum()`—return `MotionState` or `MotionGait` directly.

Example: `example/request_robot_state_example.cpp`

## Motion Control Layer

Header: `include/motion_level_control.h`

Class: `bpx_sdk::MotionLevelControl`

This layer is used to send high-level robot control commands. It inherits from `RequestRobotState`, so the same object can both send motion commands and read robot state.

Connection and setup:

```cpp
bpx_sdk::MotionLevelControl motion;
motion.setRobotIp(bpx_sdk::DEFAULT_SERVER_IP);
motion.setRobotStateUploadPort(bpx_sdk::DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
motion.setTcpLocalPort(0);
motion.setRobotStateUploadRate(100);
motion.setMotionCommandRate(50);

if (!motion.connect()) {
    return 1;
}
```

Control interfaces:

| Interface | Description |
| --- | --- |
| `setMotionCommandRate(uint16_t rate_hz)` | Sets the periodic command send rate. |
| `setVelocityControlFlag(bool enabled)` | Enables or disables the velocity control flag in outgoing packets. |
| `setZeroPositionsFlag()` | Sets the zero-position flag. The example program sends this before issuing actual motion commands. |
| `setVelocity(float x, float y, float yaw)` | Sends a body velocity and yaw rate command. |
| `setStandUp()` | Requests stand-up mode. |
| `setSitDown()` | Requests sit-down mode. |
| `setDamping()` | Requests joint damping mode. |
| `setUpright()` | Requests upright/wait mode. |

Before calling `setZeroPositionsFlag()` to zero the positions, you must confirm that the robot's foot soles, lower legs, and the connection points between the lower and upper legs are all in contact with the ground. The example program `example/motion_level_control_example.cpp` sends a zeroing command at startup, so the BPX must be in the zeroing posture before running `motion_level_control_example`.

Example: `example/motion_level_control_example.cpp`

## Joint Control Layer

Header: `include/joint_level_control.h`

Class: `bpx_sdk::JointLevelControl`

This layer is used to send direct joint control commands. It also inherits all state query interfaces from `RequestRobotState`, so the application can send joint targets and read robot state within the same loop.

A wired connection is recommended for joint-level development.

When the robot is in `DevelopingState`, high-rate joint state is uploaded through the joint state channel. `JointLevelControl` receives this data stream in the background and makes the latest packet available through the `HighRate` interfaces.

The joint control model is:

```text
torque = kp * (target_position - current_position)
       + kd * (target_velocity - current_velocity)
       + torque_feed_forward
```

Connection and setup:

```cpp
bpx_sdk::JointLevelControl joint;
joint.setRobotIp(bpx_sdk::DEFAULT_SERVER_IP);
joint.setRobotStateUploadPort(bpx_sdk::DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
joint.setTcpLocalPort(0);
joint.setRobotStateUploadRate(100);

if (!joint.connect()) {
    return 1;
}
```

Control interfaces:

| Interface | Description |
| --- | --- |
| `setJointCommand(kp, pos, kd, vel, tff)` | Sends a complete 12-DOF joint command. |
| `setJointKp(kp)` | Updates and sends joint `kp` gains. |
| `setJointPosition(pos)` | Updates and sends target joint positions. |
| `setJointKd(kd)` | Updates and sends joint `kd` gains. |
| `setJointVelocity(vel)` | Updates and sends target joint velocities. |
| `setJointTorqueFeedForward(tff)` | Updates and sends feed-forward torques. |
| `setZeroJointCommand()` | Sends a zeroed joint command. |

High-rate state interfaces:

| Interface | Description |
| --- | --- |
| `getJointPositionHighRate(float[12])` | Latest high-rate joint positions from `DevelopingState`. |
| `getJointVelocityHighRate(float[12])` | Latest high-rate joint velocities from `DevelopingState`. |
| `getJointTorqueHighRate(float[12])` | Latest high-rate joint torques from `DevelopingState`. |
| `getImuRpyHighRate(float[3])` | Latest high-rate IMU roll, pitch, and yaw. |
| `getImuQuatHighRate(float[4])` | Latest high-rate IMU quaternion. |
| `getImuAccHighRate(float[3])` | Latest high-rate IMU acceleration. |
| `getImuOmegaHighRate(float[3])` | Latest high-rate IMU angular velocity. |
| `getJointStateTimestampHighRate(float*)` | Timestamp carried in the latest high-rate packet. |
| `getJointStateSeqHighRate(uint32_t*)` | Sequence number of the latest high-rate packet. |

`example/joint_level_control_example.cpp` demonstrates how to send joint commands and periodically print the motion state, gait, battery level, IMU RPY, and the high-rate position, velocity, and torque for the first six joints.

## Requirements and Build Steps

Requirements:

- Ubuntu 22.04 or later.

Build steps:

Compile the SDK examples from within the `bpx_sdk_open` directory:

```bash
mkdir build
cd build
cmake ..
make
```

After building, the example executables are located in `build/`.

CMake automatically links the correct shared library for the current system architecture. If you have other linking or build requirements, modify `CMakeLists.txt`.

## Running the Examples

The SDK communicates with the BPX over the network. If the robot IP differs from `DEFAULT_SERVER_IP`, set the robot IP before calling `connect()`. Before running a program, it is recommended to `ping` the BPX IP from the development host to confirm network connectivity before calling `connect()`.

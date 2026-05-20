# BPX SDK Open — Capabilities Review

> SDK version: 1.0.1  
> Target hardware: BPX quadruped robot (12-DOF, 3 joints per leg × 4 legs)

---

## Robot State Telemetry

### What is available

| Category | Fields | Rate |
|---|---|---|
| Joint kinematics | Position (rad), velocity (rad/s), torque | Normal-rate (configurable, up to 100 Hz) |
| IMU | RPY (ZYX Euler), quaternion (x,y,z,w), linear acceleration, angular velocity | Normal-rate |
| Body odometry | Leg odometry (x, y, θ), current body-frame velocity | Normal-rate |
| Motion metadata | Current/last motion state (enum + raw), current/last gait (enum + raw), max velocity | Normal-rate |
| Thermal | Motor temperature × 12, driver temperature × 12 | Normal-rate |
| Power | Battery level (%), battery current | Normal-rate |
| Timing | Per-domain timestamps (joint, IMU, odometry, motion state, battery) | Normal-rate |
| High-rate joint | Position, velocity, torque × 12 | High-rate (DevelopingState only) |
| High-rate IMU | RPY, quaternion, acceleration, angular velocity | High-rate (DevelopingState only) |
| High-rate metadata | Timestamp (float), sequence number | High-rate (DevelopingState only) |

### Strengths
- **Comprehensive sensory coverage**: joint kinematics, full 6-axis IMU, odometry, thermal, and power data are all available from a single subscription.
- **Dual-rate paths**: the normal-rate path suits UI, logging, and supervisory logic; the high-rate path (available in `JointLevelControl`) suits real-time control loops that need low-latency feedback.
- **Type-safe enumerations**: `MotionState` and `MotionGait` enums prevent misinterpretation of raw integer values.
- **Named joint constants**: `JointIndex` enum and `kJointNames` array make joint-indexed arrays self-documenting.

### Limitations
- **No absolute pose or world-frame odometry**: only body-frame velocity and leg odometry are exposed. There is no fused world-frame position or orientation estimate.
- **No foot contact or ground-reaction force data**: there is no API for individual foot contact state or force/pressure sensor readings.
- **No error or fault state**: there is no API for reading hardware fault codes, estop status, or communication link health.
- **Normal-rate frequency is not guaranteed**: the `setRobotStateUploadRate` parameter requests a rate, but there is no documented guarantee or feedback on actual achieved rate.
- **High-rate data requires `DevelopingState`**: the high-rate channel is only active when the robot is in the developer mode, which limits its use in production deployments.
- **No configuration readback**: the SDK cannot query what configuration (IP, ports, rates) is active on the robot side.

---

## Motion Control

### What is available

| Command | Layer | Description |
|---|---|---|
| Stand up | `MotionLevelControl` | Transitions robot to standing/motion state |
| Sit down | `MotionLevelControl` | Transitions robot to sit-down state |
| Damping | `MotionLevelControl` | Puts all joints into passive damping mode |
| Upright/wait | `MotionLevelControl` | Requests an upright stationary stance |
| Body velocity | `MotionLevelControl` | x (forward/back), y (lateral), yaw rate (rad/s) |
| Zero positions | `MotionLevelControl` | Calibrates zero-position reference (requires specific posture) |
| 12-DOF joint command | `JointLevelControl` | PD gains + target position + target velocity + feed-forward torque per joint |
| Partial joint updates | `JointLevelControl` | Send only kp, kd, position, velocity, or torque FF individually |
| Zero joint command | `JointLevelControl` | Clears all joint targets (safety reset) |

### Strengths
- **Three control abstraction levels**: the layered design lets integrators choose the right abstraction—telemetry only, high-level motion, or full joint-level access—without unnecessary complexity at each level.
- **PD + feed-forward joint model**: the explicit `torque = kp*(q_target − q) + kd*(dq_target − dq) + tff` model is a well-understood standard in legged robotics. Exposing all five terms gives integrators full flexibility.
- **Per-parameter partial updates**: `setJointKp`, `setJointPosition`, etc. allow updating a single parameter without re-specifying all five, reducing the risk of accidental overwrites in incremental workflows.
- **Zero joint command**: `setZeroJointCommand()` provides a clean emergency reset to prevent runaway joint motion when exiting a control loop.
- **Command rate is configurable**: `setMotionCommandRate` allows the application to tune the command loop frequency to match the control design.

### Limitations
- **No gait selection**: the robot can be observed to be in Walk, Running, Bipedal, WalkPhase, PoseTracking, Flip, or WalkPeriod gaits, but there is no API to request a specific gait mode programmatically.
- **No heading/orientation control**: `setVelocity` controls body-frame linear and yaw velocity but does not support position setpoints, heading lock, or orientation commands.
- **No trajectory or waypoint API**: all motion commands are instantaneous; there is no built-in trajectory interpolation, path following, or waypoint queue. Applications must implement these themselves.
- **No force/compliance control**: `MotionLevelControl` has no interface for impedance, compliance, or explicit force-based control at the high level. Force control must be implemented manually at the joint level.
- **`setZeroPositionsFlag` has no runtime safety check**: the zeroing command requires a specific physical posture but performs no verification. Improper use can cause hardware damage.
- **Velocity-mode flag must be managed manually**: `setVelocityControlFlag` is a separate toggle not tied to `setVelocity`. Forgetting to enable it means velocity commands have no effect; forgetting to disable it when transitioning modes can cause unintended motion.
- **No acknowledgement or feedback on commands**: every control method returns `bool`, indicating only whether the packet was sent successfully—not whether the robot received or executed the command.

---

## Platform and Integration

### Strengths
- **Dual architecture support**: pre-built libraries for both x86-64 (development workstations) and AArch64 (embedded single-board computers such as Jetson or Raspberry Pi) are included.
- **Standard CMake build**: the SDK integrates with standard CMake workflows and auto-selects the correct `.so` based on the detected architecture.
- **Minimal header dependencies**: no third-party library headers are exposed, reducing dependency conflicts in larger projects.
- **Runtime IP configuration**: `setRobotIp` and port setters allow targeting different robots without recompilation.

### Limitations
- **Linux only, Ubuntu 22.04+**: no Windows, macOS, or embedded RTOS support. Older Ubuntu versions and other Linux distributions are not officially supported.
- **No Python or ROS bindings**: there are no official Python wrappers or ROS 2 packages. Integration with common robotics tooling (RViz, nav2, MoveIt) requires custom bridging.
- **Closed-source binary**: the `.so` libraries cannot be inspected, modified, or ported. Bug fixes depend entirely on the SDK vendor.
- **No versioned ABI stability guarantee**: the SDK is described as an initial implementation that "will be enriched over time." There is no documented ABI compatibility policy, which means future SDK versions may break existing binaries.
- **No simulation or mock mode**: there is no software-in-the-loop or simulated robot target, making development without physical hardware impractical.
- **Single robot per process**: the SDK design uses a global default IP and per-class connection state. Controlling multiple robots from the same process would require creating multiple instances and carefully managing port assignments.

---

## Overall Assessment

| Capability Area | Status | Notes |
|---|---|---|
| Joint kinematics telemetry | ✅ Complete | Full 12-DOF, normal + high-rate |
| IMU telemetry | ✅ Complete | Full 6-axis, normal + high-rate |
| Thermal & power monitoring | ✅ Present | Motor/driver temps, battery |
| Body velocity control | ✅ Present | x, y, yaw |
| Discrete state transitions | ✅ Present | Stand, sit, damp, upright |
| Joint-level PD+FF control | ✅ Present | All 5 parameters per joint |
| Gait selection | ❌ Missing | Read-only, cannot be set |
| Absolute world-frame pose | ❌ Missing | No SLAM, VIO, or fused pose |
| Foot contact / GRF data | ❌ Missing | No contact sensing API |
| Fault / health monitoring | ❌ Missing | No hardware error codes |
| Trajectory / waypoint API | ❌ Missing | Must be implemented by user |
| ROS 2 / Python bindings | ❌ Missing | No official wrappers |
| Simulation / mock mode | ❌ Missing | Physical hardware required |
| Multi-platform support | ⚠️ Limited | x86-64 + AArch64 Linux only |
| ABI stability guarantee | ⚠️ Undocumented | SDK described as early-stage |

The SDK is well-suited for research and prototyping tasks that need low-level joint control or high-rate feedback. It is not yet mature enough for production deployments where fault tolerance, multi-robot coordination, or simulation-based development are required.

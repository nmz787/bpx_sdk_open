# BPX SDK Open — API Review

> SDK version: 1.0.1  
> Source reviewed: `include/`, `example/`, `README.md`

---

## Overview

The SDK exposes three C++ classes in the `bpx_sdk` namespace, all sharing a common inheritance chain:

```
RequestRobotState          (state query only)
    ├── MotionLevelControl (high-level motion commands + state query)
    └── JointLevelControl  (12-DOF joint commands + high-rate state + state query)
```

Communication is over UDP (robot state upload) and TCP (control channel). Configuration constants live in `bpx_sdk_config.h`; type-safe enumerations and joint names live in `motion_types.h`.

---

## API Surface

### `RequestRobotState`

Provides ~50 getter methods across two flavours for every data field:

| Flavour | Signature pattern | Return convention |
|---|---|---|
| Output-parameter | `bool getJointPosition(float[12])` | Returns `true` on fresh data, `false` if not yet received |
| Optional helper | `std::optional<std::array<float,12>> getJointPositionArray()` | `nullopt` if not yet received |

Covered data domains: joint position/velocity/torque, IMU RPY/quaternion/acceleration/angular velocity, body velocity, leg odometry, motor/driver temperatures, motion state & gait (raw `uint8_t` and typed enum), max velocity, battery level/current, and per-domain timestamps.

### `MotionLevelControl`

Adds seven methods on top of `RequestRobotState`:

- `setMotionCommandRate(uint16_t)` — command loop frequency
- `setVelocityControlFlag(bool)` — velocity-mode toggle
- `setZeroPositionsFlag()` — zero-position calibration
- `setVelocity(float x, float y, float yaw)` — body velocity
- `setStandUp()`, `setSitDown()`, `setDamping()`, `setUpright()` — discrete state transitions

### `JointLevelControl`

Adds seven write methods:

- `setJointCommand(kp, pos, kd, vel, tff)` — full 12-DOF PD + feed-forward command
- `setJointKp / setJointKd / setJointPosition / setJointVelocity / setJointTorqueFeedForward` — partial updates
- `setZeroJointCommand()` — emergency zero

And nine high-rate read methods covering joint pos/vel/torque, IMU RPY/quaternion/acceleration/angular velocity, timestamp, and sequence number.

---

## Strengths

### 1. Clean inheritance design
`MotionLevelControl` and `JointLevelControl` both inherit from `RequestRobotState`, so a single object handles both read and write paths without duplicating state subscription code. The pattern is easy to understand and extend.

### 2. Dual getter flavours
Every state field is available both as an output-parameter getter (C-style, zero-allocation) and as an `std::optional`-returning helper (more idiomatic modern C++). This lets performance-sensitive inner loops use the former while application glue code uses the latter.

### 3. Type-safe enumerations
`MotionState` and `MotionGait` are strongly typed `enum class`, and `JointIndex` names all 12 joints. This prevents silent raw-value misuse common in robotics SDKs.

### 4. Orthogonal high-rate channel
`JointLevelControl` exposes a separate high-rate data path (driven by `DevelopingState`) alongside the normal-rate path. Separating the two prevents high-frequency joint data from contending with lower-frequency state packets at the API level.

### 5. Pimpl idiom (`class Impl`)
All three classes use the pimpl (pointer-to-implementation) idiom. This hides internal implementation detail (networking, threading, buffers) behind a stable ABI, and the pre-built `.so` libraries match this expectation.

### 6. Minimal dependency surface
The public headers only require `<array>`, `<cstdint>`, `<memory>`, and `<optional>`. No third-party library headers are exposed, keeping integration friction low.

### 7. Configurable ports and IP
`setRobotIp`, `setRobotStateUploadPort`, `setTcpLocalPort`, and `setRobotStateUploadRate` are all runtime-configurable, allowing the same binary to target different robot network configurations without recompilation.

---

## Weaknesses and Concerns

### 1. No error details on failure
Every control method returns `bool` (success/failure), and every state getter returns `bool` or `std::optional`. There is no error code, exception, or string message when something goes wrong (e.g., socket error, version mismatch, timeout). Callers must guess the root cause from logs or by re-reading source.

### 2. No documented data freshness or staleness policy
The getters return `false` / `nullopt` when "no data has been received yet," but there is no API to query *how old* the most recent packet is, nor any documented timeout after which a getter reports stale data. This can mask network failures in long-running applications.

### 3. Timestamps are per-domain, not correlated
Each data domain (joints, IMU, odometry, motion state, battery) has its own timestamp getter, but there is no mechanism to fetch a consistent snapshot of multiple domains at the same logical time. Consumers must accept potential inconsistencies across domains within a single control cycle.

### 4. `setZeroPositionsFlag()` is a latent hazard
The zero-position calibration method (`setZeroPositionsFlag`) requires the robot to be in a specific physical posture. The method name does not convey this constraint, and there is no runtime check or guard. Calling it at the wrong time can damage the hardware. The README documents the requirement, but API-level safety (e.g., requiring an explicit confirmation parameter) would be safer.

### 5. High-rate timestamp type mismatch
`getJointStateTimestampHighRate` takes a `float*`, while all normal-rate timestamp getters take `uint32_t*`. The high-rate timestamp silently loses precision for large millisecond values, and the inconsistency is a likely source of bugs for callers who assume the same type.

### 6. No gait selection API in `MotionLevelControl`
`MotionLevelControl` exposes discrete motion state commands (`setStandUp`, `setSitDown`, `setDamping`, `setUpright`) but provides no way to select a gait (Walk, Running, Bipedal, etc.) directly. The SDK can *read* the current gait, but the application has no programmatic way to request a specific gait mode.

### 7. `setVelocityControlFlag` must be managed manually
The velocity control flag must be toggled on (`true`) before `setVelocity` has effect and toggled off (`false`) when transitioning to a non-velocity mode. This state dependency is not enforced by the API and is easy to get wrong (as seen in the example where the flag is toggled inside phase transitions scattered through a switch statement).

### 8. Closed-source binary
The implementation is delivered as pre-built `.so` files (`libbpx_sdk_x86_64.so`, `libbpx_sdk_aarch64.so`). Users cannot inspect internals, contribute fixes, or port to unsupported architectures (e.g., RISC-V, Windows). The SDK is limited to Ubuntu 22.04+ on x86-64 or AArch64.

### 9. No thread-safety documentation
There is no documentation on whether getters are safe to call from one thread while the internal receive thread updates state, or whether concurrent calls to setters are safe. The pimpl pattern likely hides mutex protection, but without documentation, callers cannot safely design multi-threaded consumers.

### 10. `setRobotIp` accepts `const char*`, not `std::string`
Using raw `const char*` for the IP address is a minor footgun (no bounds checking, easy to pass a dangling pointer). An overload accepting `std::string_view` or `std::string` would be safer and consistent with the rest of the modern C++ interface.

---

## Summary Table

| Aspect | Rating | Notes |
|---|---|---|
| Ease of getting started | ✅ Good | Clear examples, minimal headers |
| Type safety | ✅ Good | `enum class`, `std::optional`, `std::array` |
| Error reporting | ⚠️ Weak | `bool`/`nullopt` only, no reason codes |
| Data freshness guarantees | ⚠️ Weak | No staleness API |
| Consistent snapshot reads | ⚠️ Weak | Per-domain timestamps only |
| Safety guards | ⚠️ Weak | `setZeroPositionsFlag` has no runtime guard |
| High-rate timestamp type | ❌ Bug-prone | `float*` vs `uint32_t*` inconsistency |
| Portability | ❌ Limited | x86-64 / AArch64 Linux only |
| Thread-safety documentation | ❌ Missing | Not documented |
| Gait control completeness | ⚠️ Partial | Readable but not settable |

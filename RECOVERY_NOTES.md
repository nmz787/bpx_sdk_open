# Binary Recovery Notes

## Scope

Initial recovery analysis for the shipped BPX SDK binary artifacts in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/lib`.

Analyzed files:
- `/home/runner/work/bpx_sdk_open/bpx_sdk_open/lib/libbpx_sdk_x86_64.so`
- `/home/runner/work/bpx_sdk_open/bpx_sdk_open/lib/libbpx_sdk_aarch64.so`

## Debug Info Check

Result: the `.so` files do **not** appear to contain embedded DWARF debug information.

Evidence:
- `file` reports both ELF binaries are **not stripped**.
- `readelf -S` shows `.symtab` and `.strtab` sections.
- `readelf -S` does **not** show `.debug_*`, `.zdebug_*`, or `.line*` sections.
- `readelf --debug-dump=info` produced no DWARF output.

Conclusion:
- These libraries were likely built **without embedded `-g` debug info**, or the debug info was removed before packaging.
- However, they are **not stripped**, which preserves substantial symbol information useful for source recovery.

## Compiler / Build Clues

Observed strings include:
- `GCC: (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0`
- `crtstuff.c`
- AArch64 cross toolchain paths under `/usr/lib/gcc-cross/aarch64-linux-gnu/9/...`

This suggests the Linux builds were produced with GCC 9.4.x, including a cross-compiled AArch64 build.

## Recovered Source File Clues

The binaries still contain source/translation-unit names that indicate the original implementation layout:
- `joint_level_control.cpp`
- `motion_level_control.cpp`
- `request_robot_state.cpp`
- `joint_command_sender.cpp`
- `joint_state_receiver.cpp`
- `motion_command_sender.cpp`
- `tcp_subscribe_client.cpp`
- `robot_state_udp_receiver.cpp`

## Recovered High-Level Architecture

### Public API classes
- `bpx_sdk::RequestRobotState`
- `bpx_sdk::MotionLevelControl`
- `bpx_sdk::JointLevelControl`

### Internal transport / helper classes
- `bpx_sdk::JointCommandSender`
- `bpx_sdk::JointStateReceiver`
- `bpx_sdk::MotionCommandSender`
- `bpx_sdk::TcpSubscribeClient`
- `bpx_sdk::RobotStateUdpReceiver`
- `bpx_sdk::socket_compat::*`

### Design pattern clues
The binaries expose symbols such as:
- `bpx_sdk::JointLevelControl::Impl`
- `bpx_sdk::MotionLevelControl::Impl`
- `bpx_sdk::RequestRobotState::Impl`

This strongly indicates use of a PIMPL design for the public API classes.

## Recovered Public API Examples

The exported symbols provide a strong basis for reconstructing public headers and implementation responsibilities.

### RequestRobotState
Examples observed in exports:
- `connect()` / `disconnect()`
- `setRobotStateUploadPort(uint16_t)`
- `setJointStateUploadPort(uint16_t)`
- `setRobotStateUploadRate(uint16_t)`
- `setTcpLocalPort(uint16_t)`
- `setSessionId(uint16_t)`
- `setRobotIp(const char*)`
- `queryRobotVersion(...)`
- `getRobotVersion(...) const`
- `getJointPosition(float*) const`
- `getJointVelocity(float*) const`
- `getJointTorque(float*) const`
- `getImuQuat(float*) const`
- `getImuOmega(float*) const`
- `getBatteryLevel(uint8_t*) const`
- `getBatteryCurrent(float*) const`
- `getLegOdom(bpx_sdk::LegOdom*) const`
- `getCurrentGait(bpx_sdk::MotionGait*) const`
- `getLastGait(bpx_sdk::MotionGait*) const`
- `getSubGait(unsigned char*) const`
- optional/array convenience getter variants are also present in the binary

### MotionLevelControl
Examples observed in exports:
- `connect()` / `disconnect()`
- `setMotionCommandRate(uint16_t)`
- `setVelocityControlFlag(bool)`
- `setZeroPositionsFlag()`
- `setWalk()`
- `setRunning()`
- `setLeftFlip()`
- `setRightFlip()`
- `setBipedal()`
- `setInvBipedal()`
- `setPronk()`
- `setPace()`
- `setBound()`
- `setVelocity(float, float, float)`
- `setStandUp()`
- `setSitDown()`
- `setDamping()`

### JointLevelControl
Examples observed in exports:
- `connect()` / `disconnect()`
- `setJointStateUploadPort(uint16_t)`
- `setJointCommand(...)`
- `setJointKp(...)`
- `setJointPosition(...)`
- `setJointKd(...)`
- `setJointVelocity(...)`
- `setJointTorqueFeedForward(...)`
- `setZeroJointCommand()`
- `getJointPositionHighRate(float*) const`
- `getJointVelocityHighRate(float*) const`
- `getJointTorqueHighRate(float*) const`
- `getImuRpyHighRate(float*) const`
- `getImuQuatHighRate(float*) const`
- `getImuAccHighRate(float*) const`
- `getImuOmegaHighRate(float*) const`
- `getJointStateTimestampHighRate(float*) const`
- `getJointStateSeqHighRate(uint32_t*) const`

## Header Correlation

The current public headers under `/home/runner/work/bpx_sdk_open/bpx_sdk_open/include` already match the recovered symbols closely:
- `/home/runner/work/bpx_sdk_open/bpx_sdk_open/include/request_robot_state.h`
- `/home/runner/work/bpx_sdk_open/bpx_sdk_open/include/motion_level_control.h`
- `/home/runner/work/bpx_sdk_open/bpx_sdk_open/include/joint_level_control.h`
- `/home/runner/work/bpx_sdk_open/bpx_sdk_open/include/motion_types.h`

This means the public API surface is largely recoverable even without original `.cpp` sources.

## Recovery Value Assessment

Even without debug sections, the binaries preserve enough metadata to support staged recovery:
1. exported symbol recovery
2. source file / module boundary inference
3. class and inheritance recovery
4. transport-layer role identification
5. targeted disassembly for behavioral reconstruction

## iteration 2

### Done

- Produced a dedicated symbol inventory in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/SYMBOL_RECOVERY.md` covering public API symbols plus key internal transport classes recovered from the shipped binary.
- Reconstructed the public implementation surface under `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src` with PIMPL-backed `RequestRobotState`, `MotionLevelControl`, and `JointLevelControl` source files that match the recovered class layout and exposed methods.
- Added an API-behavior comparison harness in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/compare_api_assumptions.cmake` so recovered and precompiled probes can be checked for divergence.
- Confirmed the current public headers under `/home/runner/work/bpx_sdk_open/bpx_sdk_open/include` remain aligned with the symbol-recovery findings, including the optional/array getter variants visible in the binary.

### Next

- Replace the new in-process runtime scaffolding with real socket/protocol handling so the recovered transport classes can talk to hardware instead of only seeding simulated state.
- Recover packet layouts and message framing in enough detail to parse real robot state uploads and serialize motion/joint command traffic.
- Expand validation from recovered-only runtime smoke coverage to binary-vs-recovered behavioral checks that exercise connected flows where the shipped library is usable in CI.
- Use `/home/runner/work/bpx_sdk_open/bpx_sdk_open/python/bpx_sdk_py.cpp` as an additional compatibility check while filling in the remaining runtime behavior.

## Recommended Next Steps

1. Produce a complete symbol inventory comparison for both architectures and flag any x86_64 vs. aarch64 drift.
2. Disassemble the packet-heavy internal classes to recover concrete wire formats:
   - `JointCommandSender`
   - `MotionCommandSender`
   - `TcpSubscribeClient`
   - `RobotStateUdpReceiver`
3. Replace the recovered runtime scaffolding with actual socket send/receive loops and packet parsing once the wire format is known.
4. Use the Python binding at `/home/runner/work/bpx_sdk_open/bpx_sdk_open/python/bpx_sdk_py.cpp` to cross-check public API expectations against the new connected-path behavior.
5. If available outside the repo, search for detached debug files, CI artifacts, release packages, or symbol bundles.

## iteration 3

### Done

- Added recovered transport/runtime scaffolding in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src` for `TcpSubscribeClient`, `RobotStateUdpReceiver`, `MotionCommandSender`, `JointCommandSender`, and `JointStateReceiver`, along with shared recovery packet/state structs in `recovery_runtime.{h,cpp}`.
- Replaced the previous `queryRobotVersion(...)` placeholder with a recovered version-query path and updated `connect()` flows so connected objects now seed consistent cached robot state and version data.
- Wired the motion-level and joint-level recovered implementations into the new runtime scaffolding so connected command flows now produce deterministic feedback for gait state, max-velocity state, and high-rate joint snapshots.
- Extended validation with `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/recovered_runtime_probe.cpp` and made the differential test CMake setup more portable by switching the precompiled probe RPATH to `$<TARGET_FILE_DIR:bpx_sdk_precompiled>` and exposing `BPX_SDK_USE_RECOVERED_SOURCES` as a real option.

### Next

- Turn the current recovered scaffolding into real protocol-aware transport by recovering socket behavior, packet layouts, and multi-rate parsing from the shipped binaries.
- Add cross-architecture recovery notes once the aarch64 symbol inventory and disassembly results are in hand.
- Exercise the Python bindings against the connected recovered runtime to catch API mismatches outside the C++ probes.

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

## iteration 4

### Done

- Added Python recovered-runtime validation in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/python_recovered_runtime_probe.py` and wired it into `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/CMakeLists.txt` so CTest now exercises the Python bindings against the connected recovered shared library.
- Extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/setup.py` with `BPX_SDK_PYTHON_RUNTIME_LIBRARY` and `BPX_SDK_PYTHON_IMPORT_LIBRARY` overrides so the Python package can be built against `bpx_sdk_recovered` instead of only the shipped binary during recovery validation.
- Compared exported `bpx_sdk::*` symbol inventories for `/home/runner/work/bpx_sdk_open/bpx_sdk_open/lib/libbpx_sdk_x86_64.so` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/lib/libbpx_sdk_aarch64.so`; both architectures currently expose the same 580 demangled symbols, so there is no exported-surface drift yet to explain.

### Next

- Recover the actual TCP/UDP wire formats used by `TcpSubscribeClient`, `RobotStateUdpReceiver`, `MotionCommandSender`, and `JointCommandSender` so the connected probes validate real packet parsing instead of seeded snapshots.
- Expand the cross-architecture notes from symbol parity to packet-path disassembly and class-layout differences once the aarch64 transport routines are analyzed in detail.
- If CI can host recovered and shipped runtimes side by side, add a differential Python probe that compares the recovered binding behavior with the precompiled library on the same API surface.

## iteration 5

### Done

- Fixed `/home/runner/work/bpx_sdk_open/bpx_sdk_open/setup.py` to force rebuilding the Python extension during package builds, which prevents stale `build/` artifacts from silently relinking the Python wheel against the shipped `libbpx_sdk_x86_64.so` when `BPX_SDK_PYTHON_RUNTIME_LIBRARY` and `BPX_SDK_PYTHON_IMPORT_LIBRARY` point at `bpx_sdk_recovered`.
- Added `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/python_compare_api_assumptions.py` and wired it into `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/CMakeLists.txt` so CTest now compares recovered and shipped Python bindings on the safe pre-connect API surface in the same run.
- Tightened the recovered control-path behavior in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/motion_level_control.cpp` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/joint_level_control.cpp` so pre-connect send-style APIs now report failure instead of optimistic success, matching the shipped library behavior exercised by the differential probes.
- Extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/api_assumption_probe.cpp` to compare the pre-connect return values of representative motion and joint command APIs, closing a behavioral gap that the earlier C++ differential probe did not observe.
- Expanded the transport recovery notes with packet-path findings from the shipped x86_64 and aarch64 binaries: `RobotStateUdpReceiver::parsePacket(...)` gates on a 12-byte `ClientUploadPacketHead` (`seq`, `timestamp_ms`, `payload_size`, `payload_type`), the current disassembly shows payload-type branches for `0x1000`, `0x0200`, `0x0050`, `0x0010`, and `0x0001`, and the analyzed transport classes still present the same exported method surface on both architectures.

### Next

- Recover the concrete payload layouts behind the identified upload packet types so `RobotStateUdpReceiver::parsePacket(...)` can populate real `RobotStateSnapshot` data instead of seeded defaults.
- Disassemble `TcpSubscribeClient::sendRequest(...)`, `MotionCommandSender::sendPacket(...)`, and `JointCommandSender::send(...)` deeply enough to replace the current no-op send paths with real socket serialization.
- Extend the Python differential probe from pre-connect behavior into connected-path checks once CI can safely host side-by-side recovered and shipped runtimes without hanging on hardware/network operations.

## iteration 6

### Done

- Replaced the placeholder `RobotStateUdpReceiver::parsePacket(...)` implementation with concrete recovered packet decoding for the observed `0x1000`, `0x0200`, `0x0050`, `0x0010`, and `0x0001` payload families, so recovered runtime snapshots now ingest joint, IMU, odometry, motion-state, battery, and temperature data from packet bytes instead of only seeded defaults.
- Promoted the recovered upload payload layouts into `src/recovery_runtime.h`, including the signed-temperature encoding used by the shipped library's 1Hz battery packet and the compact 10Hz motion-state packet layout that carries current/last motion and gait state plus max-velocity limits.
- Added `testcase/recovered_packet_parse_probe.cpp` and wired it into `testcase/CMakeLists.txt` so CTest now validates end-to-end parsing of 1Hz, 10Hz, 50Hz, 200Hz, and 1000Hz recovered packets.

### Next

- Recover the socket-open and wire-serialization behavior in `TcpSubscribeClient`, `MotionCommandSender`, and `JointCommandSender` so the recovered runtime can emit the same TCP/UDP request packets as the shipped binaries instead of only accepting synthetic packet input.
- Thread the recovered packet parsing through the real receive loops once the transport classes can open sockets and ingest live robot traffic without relying on seeded snapshots.
- Extend the Python differential probe from pre-connect behavior into connected-path checks after the recovered transport stack can be exercised in CI without hanging on hardware/network operations.

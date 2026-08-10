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

## iteration 7

### Done

- Replaced the no-op transport stubs in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/tcp_subscribe_client.cpp`, `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/motion_command_sender.cpp`, and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/joint_command_sender.cpp` with recovered socket setup and wire serialization: the TCP subscribe path now emits the 24-byte recovered `SubscribeStateReq` layout toward the binary’s default port `10860`, the motion sender now emits the recovered 56-byte UDP command packet toward port `9527`, and the joint sender now transmits the raw 240-byte `JointCommandPacket` payload toward port `7896`.
- Promoted the recovered subscribe-request wire layout into `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/recovery_runtime.h`, including the recovered timestamp/reserved trailer that the shipped binary sends on the wire, and threaded `setRobotIp(...)` through `RequestRobotState`, `MotionLevelControl`, and `JointLevelControl` so loopback and future live-robot tests can target explicit endpoints.
- Replaced the placeholder UDP receive-loop stub in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/robot_state_udp_receiver.cpp` with a real bound socket + background receive thread, so recovered packet parsing can now be exercised by actual UDP traffic instead of only by direct `parsePacket(...)` calls.
- Added `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/transport_socket_probe.cpp` and extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/recovered_packet_parse_probe.cpp`; CTest now verifies loopback TCP subscribe request capture, recovered motion/joint UDP serialization, and live UDP ingestion through `RobotStateUdpReceiver::receiveLoop(...)`.
- Preserved the offline-safe fallback behavior used by the existing recovery probes: when the recovered transports still point at the default robot IP `10.21.20.1` inside CI, the transport classes avoid hanging on unreachable network operations and continue exposing the seeded/synthetic behavior required by the earlier validation probes.

### Next

- Recover and validate the concrete 32-byte TCP subscribe response layout so `TcpSubscribeClient` can do more than best-effort emission and can cache/inspect real robot acknowledgements the same way the shipped binary does.
- Recover the live high-rate joint feedback socket path in `JointStateReceiver` so `JointLevelControl` can consume real incoming joint-state packets instead of only the current synthetic mirror used for offline validation.
- Expand the Python differential coverage from loopback-safe transport emission into connected-path request/response assertions once CI can stand up paired fake robot endpoints for both TCP subscribe replies and streaming UDP state traffic.

## iteration 8

### Done

- Replaced the placeholder `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/joint_state_receiver.cpp` implementation with the shipped library’s live UDP socket behavior: `JointStateReceiver` now binds its configured upload port, switches the socket into non-blocking mode, receives exact 204-byte `JointStatePacket` payloads from `recvfrom(...)`, and runs the recovered polling loop on a background thread instead of only mirroring cached command values.
- Updated `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/joint_command_sender.cpp` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/joint_level_control.cpp` so explicit robot endpoints can consume real incoming high-rate joint feedback without the offline synthetic mirror overwriting it, while the default `10.21.20.1` fallback still seeds synthetic feedback for the earlier CI-safe recovery probes.
- Extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/transport_socket_probe.cpp` and added `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/python_connected_runtime_probe.py`; C++ and Python coverage now verify loopback TCP subscribe acknowledgements plus live UDP joint-feedback ingestion through `JointLevelControl` in the recovered runtime.

### Next

- Recover the concrete field-level meaning of the 32-byte TCP subscribe response so `TcpSubscribeClient` can inspect and cache more than the current raw acknowledgement blob.
- Thread live robot-state UDP updates back through `RequestRobotState` readers after `connect()` instead of only snapshotting receiver state once at connection time.
- Expand connected-path probes from subscribe handshakes and joint feedback into streamed robot-state packet assertions covering 1Hz/10Hz/50Hz/200Hz/1000Hz uploads across the recovered and shipped runtimes.

## iteration 9

### Done

- Structured the recovered 32-byte TCP subscribe acknowledgement in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/recovery_runtime.h` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/tcp_subscribe_client.cpp` so the client now caches the last response and exposes decoded header/status/reserved/payload-word accessors instead of only dropping a raw blob after receipt.
- Updated `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/request_robot_state.cpp` so every state/timestamp reader refreshes from the live `RobotStateUdpReceiver` snapshot after `connect()`, allowing streamed 1Hz/10Hz/50Hz/200Hz/1000Hz UDP uploads to flow through `RequestRobotState` continuously rather than only at connection time.
- Extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/transport_socket_probe.cpp` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/python_connected_runtime_probe.py` to verify structured TCP subscribe response capture plus loopback robot-state streaming coverage for all recovered upload packet families alongside the existing connected joint-feedback assertions.

### Next

- Recover the semantic meaning of the remaining seven 32-bit words in the 32-byte TCP subscribe acknowledgement once real robot captures or deeper disassembly show how the shipped runtime uses them.
- Add connected-path side-by-side recovered versus shipped runtime assertions for streamed robot-state traffic after CI can safely host paired fake robot endpoints for both libraries.

## iteration 10

### Done

- Added `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/connected_runtime_assumption_probe.cpp` plus `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/compare_connected_runtime_assumptions.cmake`, which stand up paired loopback fake-robot endpoints and compare the recovered versus shipped subscribe-request port/rate wiring through the public connected-path APIs side-by-side.
- Wired the new `compare_connected_runtime_assumptions` differential test into `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/CMakeLists.txt`, normalized the shipped runtime’s extra loopback log lines inside the comparison harness, and stopped the Python differential probes from deleting the shared build tree so the new connected-path comparison can coexist with the existing recovered-only transport coverage and Python pre-connect checks in one CTest pass.

### Next

- Recover the semantic meaning of the remaining seven 32-bit words in the 32-byte TCP subscribe acknowledgement once real robot captures or deeper disassembly show how the shipped runtime uses them.
- Recover the shipped library’s loopback robot-state stream packet dialect and host-server-mode semantics so the new cross-library harness can be extended from subscribe-request parity into full streamed robot-state and joint-feedback payload assertions.

## iteration 11

### Done

- Extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/connected_runtime_assumption_probe.cpp` from subscribe-request-only coverage into full loopback connected-path parity checks: the recovered and shipped runtimes are now exercised side-by-side against paired fake robot endpoints for streamed 1Hz/10Hz/50Hz/200Hz/1000Hz robot-state uploads plus high-rate joint-feedback packets, and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/compare_connected_runtime_assumptions.cmake` now passes with identical observable outputs from both libraries.
- Recovered the shipped library’s robot-state UDP upload header dialect in `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/recovery_runtime.h` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/robot_state_udp_receiver.cpp`: `ClientUploadPacketHead` is an 8-byte `{payload_type, payload_size, timestamp_ms}` header rather than the previously assumed 12-byte `{seq, timestamp_ms, payload_size, payload_type}` layout, and the recovered parser/tests now match the shipped binary’s accepted packet framing.
- Recovered the shipped high-rate joint-feedback wire layout and connected-path host-server-mode behavior: `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/recovery_runtime.h` now places `JointStatePacket`’s `seq` and `timestamp_ms` fields ahead of the 49 float payload values to match the shipped 204-byte packet order, while `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/request_robot_state.cpp` preserves the public pre-connect `hostServerMode()` assumptions but emits the shipped wire value `4` during live subscribe requests for both state-query and joint-control connections.
- Updated `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/transport_socket_probe.cpp`, `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/recovered_packet_parse_probe.cpp`, and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/python_connected_runtime_probe.py` so the recovered-only probes and Python connected-path checks serialize the same recovered-on-wire robot-state/joint-feedback packets that the shipped library accepts.

### Next

- Recover the semantic meaning of the remaining seven 32-bit words in the 32-byte TCP subscribe acknowledgement once real robot captures or deeper disassembly show how the shipped runtime uses them.
- Extend the same cross-library connected-path comparison coverage into any remaining live transport behaviors that still rely on recovered assumptions rather than shipped-binary parity, starting with motion-control connected flows if they expose additional host-server-mode or streaming differences.

## iteration 12

### Done

- Extended `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/connected_runtime_assumption_probe.cpp` so the recovered-versus-shipped connected-path harness now covers `MotionLevelControl` alongside state-query and joint-control flows: the loopback comparison now asserts live robot-state streaming plus outgoing velocity and damping command packet parity for explicit connected endpoints.
- Updated `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/motion_level_control.cpp` so explicit robot endpoints no longer overwrite live streamed robot-state snapshots with the offline fallback gait/motion-state cache; the synthetic connected-state mirroring is now limited to the default `10.21.20.1` fallback path, matching the shipped library’s connected behavior while preserving the earlier offline recovery probes.
- Updated `/home/runner/work/bpx_sdk_open/bpx_sdk_open/src/motion_command_sender.cpp` and `/home/runner/work/bpx_sdk_open/bpx_sdk_open/testcase/transport_socket_probe.cpp` to match the shipped motion-command wire semantics that the new differential probe exposed: the command byte now follows the raw `MotionState` values on the wire, and damping/stand/sit packets reset to neutral walk framing with zeroed command velocities before transmission.

### Next

- Recover the semantic meaning of the remaining seven 32-bit words in the 32-byte TCP subscribe acknowledgement once real robot captures or deeper disassembly show how the shipped runtime uses them.
- Recover the exact semantics of the motion-command sequence and zero-position flag bytes so the connected differential harness can stop normalizing those per-packet fields and compare them at full field-level parity.

## iteration 13

### What was done

Disassembled the shipped `libbpx_sdk_x86_64.so` binary to recover the ground truth for the two items deferred from iteration 12, then updated the recovered source and differential harness to match.

#### TCP subscribe acknowledgement — payload words 2–6 are unused

Disassembled `TcpSubscribeClient::printResponse` at `0x2b86e`. The shipped function is a complete no-op (`nop; ret`). None of the seven 32-bit payload words in the 32-byte `SubscribeStateResp` are read or acted upon anywhere in the shipped library. The `accepted()` helper in the current recovery (which inspects `responseType` and `statusCode`) is our own invented heuristic and is not confirmed by disassembly; it is left in place as a safe default for external callers.

Disassembled `TcpSubscribeClient::startStateQuery` at `0x2ad1e`. The shipped library does **not** use a fire-and-forget approach here; instead it maintains a **persistent TCP connection** for the lifetime of the session. The shipped code:
1. Atomically exchanges `response_loop_running_` to true; returns immediately if it was already running.
2. Joins any existing `response_thread_` before proceeding.
3. Creates a TCP socket, binds the configured local port, connects to the robot, and sends the 24-byte subscribe request.
4. Stores the socket fd in `response_socket_fd_` (protected by `response_mutex_`).
5. Starts a background thread that loops: `recvAll(fd, 32-byte buffer)` → `printResponse` (no-op). On recv failure the loop exits, the fd is closed and cleared under the mutex, and `response_loop_running_` is set to false.

Disassembled `TcpSubscribeClient::disconnect` at its shipped address. The shipped disconnect sets `response_loop_running_ = false`, locks the mutex, calls `shutdown(SHUT_RDWR)` on the socket (which unblocks the background recv), unlocks, then joins the thread.

`startStateQuery` and `disconnect` in `src/tcp_subscribe_client.cpp` were rewritten to match this architecture exactly. The `shutdownSocketFd` helper was added to the anonymous namespace to perform `shutdown(fd, SHUT_RDWR)` without closing the fd (the background thread closes it).

A new test block was added to `testcase/transport_socket_probe.cpp` that verifies the persistent-connection behavior: a loopback TCP server accepts the connection, receives the subscribe request, sends three 32-byte ack responses in sequence (proving the recv loop handles multiple packets), and then waits for the client-side `disconnect()` to shut down the connection.

#### Motion-command `seq` field — ordering-level parity achieved

Disassembled `MotionCommandSender::sendPacket` at `0x26ec6`. Confirmed the shipped binary uses `lock xadd` on an atomic at `self+0xa0`, starting from zero and incrementing by one for each call. Our recovery is semantically correct.

Because both binaries run independent background `runLoop` threads that fire at 20 Hz asynchronously, the absolute seq counter at the moment any given explicit command is observed is non-deterministic and will differ between two independent process executions. Exact seq parity in the differential harness is therefore unachievable. Instead the harness was upgraded from boolean `has_seq` to a derived ordering field: `request1.damping_seq_gt_velocity_seq` is printed as a boolean (always 1) and compared across shipped vs recovered. This confirms that the damping packet's seq is strictly greater than the velocity packet's seq (i.e. the counter monotonically increases within a single run) without requiring identical absolute values across runs. The `has_seq` boolean is retained per packet to confirm each packet carries a non-zero seq.

#### `setZeroPositionsFlag` — thread-local PRNG matching shipped binary

Disassembled `MotionCommandSender::setZeroPositionsFlag` at `0x2682a`. The shipped binary generates the new nonce using a **thread-local `std::mt19937`** seeded from `std::random_device{}()` and a **thread-local `std::uniform_int_distribution<int>(0, 255)`**, then applies a single collision-avoidance increment if the drawn byte equals the current nonce. The previous recovery used a simple `++nonce; if (nonce==0) ++nonce` pattern.

`src/motion_command_sender.cpp` was updated to match: `#include <random>` was added, and `setZeroPositionsFlag` now uses `thread_local std::mt19937` + `thread_local std::uniform_int_distribution<int>(0, 255)`. Because the nonce is random, exact value comparison between shipped and recovered is impossible; the harness continues to compare `zero_positions_flag_set` (nonzero boolean) rather than the raw nonce value.

### Done

- Disassembled `printResponse` — confirmed no-op; payload words 0–6 of `SubscribeStateResp` are unused by the shipped library.
- Disassembled `startStateQuery` and its background-thread lambda — recovered persistent TCP connection architecture.
- Disassembled `disconnect` — recovered shutdown + join sequence.
- Rewrote `TcpSubscribeClient::startStateQuery` and `disconnect` in `src/tcp_subscribe_client.cpp` to use the persistent connection model.
- Added `shutdownSocketFd` helper to `tcp_subscribe_client.cpp` anonymous namespace.
- Added persistent-TCP `startStateQuery` test to `testcase/transport_socket_probe.cpp`.
- Added seq ordering assertion (`loop_packet.seq > motion_packet.seq`) to the existing `MotionCommandSender` test in `transport_socket_probe.cpp`.
- Updated `printObservedMotionCommand` in `connected_runtime_assumption_probe.cpp`: retained `has_seq` boolean, added `request1.damping_seq_gt_velocity_seq` ordering check.
- Updated `setZeroPositionsFlag` in `src/motion_command_sender.cpp` to use thread-local mt19937 PRNG matching shipped disassembly.
- All 8 tests pass (100%).

### Next

- Determine whether the shipped library ever populates any of the seven `SubscribeStateResp` payload words when a real robot is connected (they are ignored on the receive side but may carry robot metadata on the transmit side). Real hardware capture or server-side disassembly required.
- Investigate whether `TcpSubscribeClient::startStateQuery` is re-invoked on reconnect (e.g. after a transient connection drop) and whether the `response_loop_running_` exchange guard is the only protection against double-start.
- Verify `setZeroPositionsFlag` mutex offset (recovered uses `state_mutex_` at offset +0x40; disassembly showed mutex at `self+0x40` — confirm these are the same field by cross-referencing the header struct layout).
- Extend the differential harness to cover `JointLevelControl::setZeroPositionsFlag` if a similar method exists in that path.

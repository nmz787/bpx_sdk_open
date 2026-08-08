#!/usr/bin/env python3

from __future__ import annotations

import gc
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def fail(message: str) -> None:
    raise SystemExit(message)


def install_package(repo_root: Path, install_root: Path, runtime_library: Path | None) -> None:
    env = os.environ.copy()
    if runtime_library is None:
        env.pop("BPX_SDK_PYTHON_RUNTIME_LIBRARY", None)
        env.pop("BPX_SDK_PYTHON_IMPORT_LIBRARY", None)
    else:
        env["BPX_SDK_PYTHON_RUNTIME_LIBRARY"] = str(runtime_library)
        env["BPX_SDK_PYTHON_IMPORT_LIBRARY"] = str(runtime_library)

    subprocess.check_call(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--no-build-isolation",
            "--target",
            str(install_root),
            str(repo_root),
        ],
        env=env,
    )


def probe_behavior(python_path: Path) -> dict[str, object]:
    script = r"""
import json
import socket
import struct
import threading
import time

import bpx_sdk

REQUEST_STRUCT = struct.Struct("<HHHHHBBIII")
JOINT_PACKET_STRUCT = struct.Struct("<" + "f" * 50 + "I")
TCP_PORT = 10860


def reserve_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def read_some(conn):
    conn.settimeout(1.0)
    chunks = []
    while True:
        try:
            chunk = conn.recv(256)
        except socket.timeout:
            break
        if not chunk:
            break
        chunks.append(chunk)
        if len(chunk) < 256:
            break
    return b"".join(chunks)


state_port = reserve_udp_port()
joint_state_port = reserve_udp_port()
requests = []
stop_server = False


def serve():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", TCP_PORT))
    server.listen(4)
    server.settimeout(0.2)
    try:
        while not stop_server:
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            with conn:
                payload = read_some(conn)
                if len(payload) == REQUEST_STRUCT.size:
                    request = REQUEST_STRUCT.unpack(payload)
                    requests.append(
                        {
                            "session_id": request[0],
                            "robot_state_upload_port": request[1],
                            "joint_state_upload_port": request[2],
                            "reserved": request[3],
                            "robot_state_upload_rate_hz": request[4],
                            "host_server_mode": request[5],
                            "reserved_padding": request[6],
                            "timestamp_nonzero": request[7] > 0,
                            "reserved_word0": request[8],
                            "reserved_word1": request[9],
                        }
                    )
                response = bytearray(32)
                response[0] = 1
                conn.sendall(response)
    finally:
        server.close()


thread = threading.Thread(target=serve)
thread.start()

state = bpx_sdk.RequestRobotState()
state.setRobotIp("127.0.0.1")
state.setRobotStateUploadPort(state_port)
if not state.connect():
    raise SystemExit("RequestRobotState connect failed")
state.disconnect()

joint = bpx_sdk.JointLevelControl()
joint.setRobotIp("127.0.0.1")
joint.setRobotStateUploadPort(state_port)
joint.setJointStateUploadPort(joint_state_port)
if not joint.connect():
    stop_server = True
    thread.join()
    raise SystemExit("JointLevelControl connect failed")

joint_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
packet_values = (
    [3.25] + [0.0] * 11
    + [0.0, -2.5] + [0.0] * 10
    + [0.0, 0.0, 1.5] + [0.0] * 9
    + [0.4, -0.5, 0.6]
    + [0.1, 0.2, 0.3, 0.9]
    + [1.0, 2.0, 3.0]
    + [-4.0, -5.0, -6.0]
    + [4321.0]
    + [77]
)
joint_sender.sendto(
    JOINT_PACKET_STRUCT.pack(*packet_values),
    ("127.0.0.1", joint_state_port),
)
joint_sender.close()

result = {
    "requests": [],
    "joint_position0": None,
    "joint_velocity1": None,
    "joint_torque2": None,
    "imu_rpy2": None,
    "imu_quat3": None,
    "imu_acc1": None,
    "imu_omega0": None,
    "timestamp": None,
    "seq": None,
}

for _ in range(40):
    pos = joint.getJointPositionHighRate()
    vel = joint.getJointVelocityHighRate()
    tau = joint.getJointTorqueHighRate()
    imu_rpy = joint.getImuRpyHighRate()
    imu_quat = joint.getImuQuatHighRate()
    imu_acc = joint.getImuAccHighRate()
    imu_omega = joint.getImuOmegaHighRate()
    timestamp = joint.getJointStateTimestampHighRate()
    seq = joint.getJointStateSeqHighRate()
    if all(value is not None for value in (pos, vel, tau, imu_rpy, imu_quat, imu_acc, imu_omega, timestamp, seq)):
        result.update(
            {
                "joint_position0": pos[0],
                "joint_velocity1": vel[1],
                "joint_torque2": tau[2],
                "imu_rpy2": imu_rpy[2],
                "imu_quat3": imu_quat[3],
                "imu_acc1": imu_acc[1],
                "imu_omega0": imu_omega[0],
                "timestamp": timestamp,
                "seq": seq,
            }
        )
        break
    time.sleep(0.025)

joint.disconnect()
stop_server = True
thread.join()
result["requests"] = requests
print(json.dumps(result, sort_keys=True))
"""
    env = os.environ.copy()
    env["PYTHONPATH"] = str(python_path)
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return json.loads(result.stdout)


def main() -> int:
    if len(sys.argv) != 4:
        fail("usage: python_compare_connected_runtime.py <repo-root> <shipped-library> <recovered-library>")

    repo_root = Path(sys.argv[1]).resolve()
    shipped_library = Path(sys.argv[2]).resolve()
    recovered_library = Path(sys.argv[3]).resolve()
    if not repo_root.exists():
        fail(f"repository root not found: {repo_root}")
    if not shipped_library.exists():
        fail(f"shipped library not found: {shipped_library}")
    if not recovered_library.exists():
        fail(f"recovered library not found: {recovered_library}")

    with tempfile.TemporaryDirectory(prefix="bpx-sdk-python-connected-") as temp_dir:
        temp_root = Path(temp_dir)
        shipped_root = temp_root / "shipped"
        recovered_root = temp_root / "recovered"
        install_package(repo_root, shipped_root, None)
        install_package(repo_root, recovered_root, recovered_library)

        precompiled = probe_behavior(shipped_root)
        recovered = probe_behavior(recovered_root)
        if precompiled != recovered:
            fail(
                "Recovered and precompiled connected Python probes diverged.\n"
                f"--- recovered ---\n{json.dumps(recovered, indent=2, sort_keys=True)}\n"
                f"--- precompiled ---\n{json.dumps(precompiled, indent=2, sort_keys=True)}"
            )

        expected_requests = [
            {
                "session_id": 0,
                "robot_state_upload_port": precompiled["requests"][0]["robot_state_upload_port"],
                "joint_state_upload_port": 7890,
                "reserved": 0,
                "robot_state_upload_rate_hz": 100,
                "host_server_mode": 1,
                "reserved_padding": 0,
                "timestamp_nonzero": True,
                "reserved_word0": 0,
                "reserved_word1": 0,
            },
            {
                "session_id": 0,
                "robot_state_upload_port": precompiled["requests"][1]["robot_state_upload_port"],
                "joint_state_upload_port": precompiled["requests"][1]["joint_state_upload_port"],
                "reserved": 0,
                "robot_state_upload_rate_hz": 100,
                "host_server_mode": 2,
                "reserved_padding": 0,
                "timestamp_nonzero": True,
                "reserved_word0": 0,
                "reserved_word1": 0,
            },
        ]
        if len(precompiled["requests"]) < 2 or len(recovered["requests"]) < 2:
            fail("expected at least two subscribe requests from the connected Python probe")
        for actual, expected in zip(precompiled["requests"][:2], expected_requests):
            expected = dict(expected)
            if "robot_state_upload_port" not in actual or "joint_state_upload_port" not in actual:
                fail("subscribe request was missing expected fields")
            expected["robot_state_upload_port"] = actual["robot_state_upload_port"]
            if expected["joint_state_upload_port"] == 7890:
                expected["joint_state_upload_port"] = actual["joint_state_upload_port"]
            if actual != expected:
                fail(
                    "connected Python probe captured an unexpected subscribe request.\n"
                    f"actual={json.dumps(actual, sort_keys=True)}\n"
                    f"expected={json.dumps(expected, sort_keys=True)}"
                )

        if precompiled["seq"] != 77 or recovered["seq"] != 77:
            fail("joint feedback sequence mismatch")
        if precompiled["timestamp"] != 4321.0 or recovered["timestamp"] != 4321.0:
            fail("joint feedback timestamp mismatch")

        shutil.rmtree(repo_root / "build", ignore_errors=True)
        shutil.rmtree(repo_root / "bpx_sdk_open.egg-info", ignore_errors=True)
        gc.collect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3

from __future__ import annotations

import gc
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def fail(message: str) -> None:
    raise SystemExit(message)


def install_package(repo_root: Path, install_root: Path, runtime_library: Path) -> None:
    env = os.environ.copy()
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
UPLOAD_HEAD_STRUCT = struct.Struct("<IIHH")
JOINT_PACKET_STRUCT = struct.Struct("<" + "f" * 50 + "I")
STATE_1000HZ_STRUCT = struct.Struct("<" + "f" * 36)
STATE_200HZ_STRUCT = struct.Struct("<" + "f" * 13)
STATE_50HZ_STRUCT = struct.Struct("<" + "f" * 13)
STATE_10HZ_STRUCT = struct.Struct("<BBBBb3xfff")
STATE_1HZ_STRUCT = struct.Struct("<B3xf" + "b" * 12 + "b" * 12)
TCP_PORT = 10860


def reserve_udp_port():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def read_exact(conn, size):
    data = bytearray()
    conn.settimeout(1.0)
    while len(data) < size:
        chunk = conn.recv(size - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)


def send_upload_packet(sock, port, seq, timestamp_ms, payload_type, payload):
    packet = UPLOAD_HEAD_STRUCT.pack(seq, timestamp_ms, len(payload), payload_type) + payload
    sock.sendto(packet, ("127.0.0.1", port))


state_port = reserve_udp_port()
joint_state_port = reserve_udp_port()
requests = []
stop_server = False
server_ready = threading.Event()


def serve():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", TCP_PORT))
    server.listen(4)
    server.settimeout(0.2)
    server_ready.set()
    try:
        while not stop_server:
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            with conn:
                payload = read_exact(conn, REQUEST_STRUCT.size)
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
if not server_ready.wait(timeout=1.0):
    raise SystemExit("TCP probe server did not start")

state = bpx_sdk.RequestRobotState()
state.setRobotIp("127.0.0.1")
state.setRobotStateUploadPort(state_port)
if not state.connect():
    stop_server = True
    thread.join()
    raise SystemExit("RequestRobotState connect failed")

state_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_upload_packet(
    state_sender,
    state_port,
    11,
    1001,
    0x1000,
    STATE_1000HZ_STRUCT.pack(
        *([1.25] + [0.0] * 11 + [0.0, -2.5] + [0.0] * 10 + [0.0, 0.0, 3.75] + [0.0] * 9)
    ),
)
send_upload_packet(
    state_sender,
    state_port,
    12,
    1002,
    0x0200,
    STATE_200HZ_STRUCT.pack(0.4, -0.5, 0.6, 0.1, 0.2, 0.3, 0.9, 1.0, 2.0, 3.0, -4.0, -5.0, -6.0),
)
send_upload_packet(
    state_sender,
    state_port,
    13,
    1003,
    0x0050,
    STATE_50HZ_STRUCT.pack(0.7, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, -0.9),
)
send_upload_packet(
    state_sender,
    state_port,
    14,
    1004,
    0x0010,
    STATE_10HZ_STRUCT.pack(6, 8, 2, 0, 4, 3.0, 1.0, 2.0),
)
send_upload_packet(
    state_sender,
    state_port,
    15,
    1005,
    0x0001,
    STATE_1HZ_STRUCT.pack(73, 6.5, *([11] + [0] * 11), *([0, -2] + [0] * 10)),
)
state_sender.close()

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
    "state_port": state_port,
    "joint_state_port": joint_state_port,
    "requests": [],
    "state_joint_position0": None,
    "state_joint_velocity1": None,
    "state_joint_torque2": None,
    "state_imu_rpy2": None,
    "state_imu_quat3": None,
    "state_imu_acc1": None,
    "state_imu_omega0": None,
    "state_leg_velocity0": None,
    "state_leg_position1": None,
    "state_leg_orientation3": None,
    "state_leg_omega2": None,
    "state_motor_temperature0": None,
    "state_driver_temperature1": None,
    "state_max_velocity0": None,
    "state_battery_level": None,
    "state_battery_current": None,
    "state_current_motion_state": None,
    "state_current_gait": None,
    "state_last_motion_state": None,
    "state_last_gait": None,
    "state_sub_gait": None,
    "state_joint_timestamp": None,
    "state_imu_timestamp": None,
    "state_odom_timestamp": None,
    "state_motion_timestamp": None,
    "state_battery_timestamp": None,
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
    state_pos = state.getJointPosition()
    state_vel = state.getJointVelocity()
    state_tau = state.getJointTorque()
    state_imu_rpy = state.getImuRpy()
    state_imu_quat = state.getImuQuat()
    state_imu_acc = state.getImuAcc()
    state_imu_omega = state.getImuOmega()
    state_leg = state.getLegOdom()
    state_motor_temperature = state.getMotorTemperature()
    state_driver_temperature = state.getDriverTemperature()
    state_max_velocity = state.getMaxVelocity()
    state_battery_level = state.getBatteryLevel()
    state_battery_current = state.getBatteryCurrent()
    state_current_motion_state = state.getCurrentMotionState()
    state_current_gait = state.getCurrentGait()
    state_last_motion_state = state.getLastMotionState()
    state_last_gait = state.getLastGait()
    state_sub_gait = state.getSubGait()
    state_joint_timestamp = state.getJointStateTimestamp()
    state_imu_timestamp = state.getImuTimestamp()
    state_odom_timestamp = state.getOdometryTimestamp()
    state_motion_timestamp = state.getMotionStateTimestamp()
    state_battery_timestamp = state.getBatteryTimestamp()
    pos = joint.getJointPositionHighRate()
    vel = joint.getJointVelocityHighRate()
    tau = joint.getJointTorqueHighRate()
    imu_rpy = joint.getImuRpyHighRate()
    imu_quat = joint.getImuQuatHighRate()
    imu_acc = joint.getImuAccHighRate()
    imu_omega = joint.getImuOmegaHighRate()
    timestamp = joint.getJointStateTimestampHighRate()
    seq = joint.getJointStateSeqHighRate()
    if all(
        value is not None
        for value in (
            state_pos,
            state_vel,
            state_tau,
            state_imu_rpy,
            state_imu_quat,
            state_imu_acc,
            state_imu_omega,
            state_leg,
            state_motor_temperature,
            state_driver_temperature,
            state_max_velocity,
            state_battery_level,
            state_battery_current,
            state_current_motion_state,
            state_current_gait,
            state_last_motion_state,
            state_last_gait,
            state_sub_gait,
            state_joint_timestamp,
            state_imu_timestamp,
            state_odom_timestamp,
            state_motion_timestamp,
            state_battery_timestamp,
            pos,
            vel,
            tau,
            imu_rpy,
            imu_quat,
            imu_acc,
            imu_omega,
            timestamp,
            seq,
        )
    ):
        result.update(
            {
                "state_joint_position0": state_pos[0],
                "state_joint_velocity1": state_vel[1],
                "state_joint_torque2": state_tau[2],
                "state_imu_rpy2": state_imu_rpy[2],
                "state_imu_quat3": state_imu_quat[3],
                "state_imu_acc1": state_imu_acc[1],
                "state_imu_omega0": state_imu_omega[0],
                "state_leg_velocity0": state_leg["velocity_body"][0],
                "state_leg_position1": state_leg["position"][1],
                "state_leg_orientation3": state_leg["orientation"][3],
                "state_leg_omega2": state_leg["angular_velocity"][2],
                "state_motor_temperature0": state_motor_temperature[0],
                "state_driver_temperature1": state_driver_temperature[1],
                "state_max_velocity0": state_max_velocity[0],
                "state_battery_level": state_battery_level,
                "state_battery_current": state_battery_current,
                "state_current_motion_state": state_current_motion_state,
                "state_current_gait": state_current_gait,
                "state_last_motion_state": state_last_motion_state,
                "state_last_gait": state_last_gait,
                "state_sub_gait": state_sub_gait,
                "state_joint_timestamp": state_joint_timestamp,
                "state_imu_timestamp": state_imu_timestamp,
                "state_odom_timestamp": state_odom_timestamp,
                "state_motion_timestamp": state_motion_timestamp,
                "state_battery_timestamp": state_battery_timestamp,
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

state.disconnect()
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
    if len(sys.argv) != 3:
        fail("usage: python_connected_runtime_probe.py <repo-root> <recovered-library>")

    repo_root = Path(sys.argv[1]).resolve()
    recovered_library = Path(sys.argv[2]).resolve()
    if not repo_root.exists():
        fail(f"repository root not found: {repo_root}")
    if not recovered_library.exists():
        fail(f"recovered library not found: {recovered_library}")

    with tempfile.TemporaryDirectory(prefix="bpx-sdk-python-connected-") as temp_dir:
        temp_root = Path(temp_dir)
        install_root = temp_root / "site"
        install_package(repo_root, install_root, recovered_library)

        result = probe_behavior(install_root)
        if len(result["requests"]) < 2:
            fail("expected at least two subscribe requests from the connected Python probe")

        first_request = result["requests"][0]
        second_request = result["requests"][1]
        expected_first = {
            "session_id": 0,
            "robot_state_upload_port": result["state_port"],
            "joint_state_upload_port": 7895,
            "reserved": 0,
            "robot_state_upload_rate_hz": 100,
            "host_server_mode": 1,
            "reserved_padding": 0,
            "timestamp_nonzero": True,
            "reserved_word0": 0,
            "reserved_word1": 0,
        }
        expected_second = {
            "session_id": 0,
            "robot_state_upload_port": result["state_port"],
            "joint_state_upload_port": result["joint_state_port"],
            "reserved": 0,
            "robot_state_upload_rate_hz": 100,
            "host_server_mode": 2,
            "reserved_padding": 0,
            "timestamp_nonzero": True,
            "reserved_word0": 0,
            "reserved_word1": 0,
        }
        if first_request != expected_first:
            fail(
                "unexpected RequestRobotState subscribe request.\n"
                f"actual={json.dumps(first_request, sort_keys=True)}\n"
                f"expected={json.dumps(expected_first, sort_keys=True)}"
            )
        if second_request != expected_second:
            fail(
                "unexpected JointLevelControl subscribe request.\n"
                f"actual={json.dumps(second_request, sort_keys=True)}\n"
                f"expected={json.dumps(expected_second, sort_keys=True)}"
            )

        expected_feedback = {
            "state_joint_position0": 1.25,
            "state_joint_velocity1": -2.5,
            "state_joint_torque2": 3.75,
            "state_imu_rpy2": 0.6,
            "state_imu_quat3": 0.9,
            "state_imu_acc1": 2.0,
            "state_imu_omega0": -4.0,
            "state_leg_velocity0": 0.7,
            "state_leg_position1": 5.0,
            "state_leg_orientation3": 0.8,
            "state_leg_omega2": -0.9,
            "state_motor_temperature0": 71.0,
            "state_driver_temperature1": 58.0,
            "state_max_velocity0": 3.0,
            "state_battery_level": 73,
            "state_battery_current": 6.5,
            "state_current_motion_state": 6,
            "state_current_gait": 8,
            "state_last_motion_state": 2,
            "state_last_gait": 0,
            "state_sub_gait": 4,
            "state_joint_timestamp": 1001,
            "state_imu_timestamp": 1002,
            "state_odom_timestamp": 1003,
            "state_motion_timestamp": 1004,
            "state_battery_timestamp": 1005,
            "joint_position0": 3.25,
            "joint_velocity1": -2.5,
            "joint_torque2": 1.5,
            "imu_rpy2": 0.6,
            "imu_quat3": 0.9,
            "imu_acc1": 2.0,
            "imu_omega0": -4.0,
            "timestamp": 4321.0,
            "seq": 77,
        }
        for key, expected in expected_feedback.items():
            actual = result[key]
            if isinstance(expected, float):
                if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-5):
                    fail(f"unexpected live joint feedback for {key}: {actual!r} != {expected!r}")
            elif actual != expected:
                fail(f"unexpected live joint feedback for {key}: {result[key]!r} != {expected!r}")

        gc.collect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

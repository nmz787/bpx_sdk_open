#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def close_enough(lhs: float, rhs: float) -> bool:
    return math.isclose(lhs, rhs, rel_tol=0.0, abs_tol=1e-5)


def fail(message: str) -> None:
    raise SystemExit(message)


def install_package(repo_root: Path, recovered_library: Path, install_root: Path) -> None:
    env = os.environ.copy()
    env["BPX_SDK_PYTHON_RUNTIME_LIBRARY"] = str(recovered_library)
    env["BPX_SDK_PYTHON_IMPORT_LIBRARY"] = str(recovered_library)
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


def main() -> int:
    if len(sys.argv) != 3:
        fail("usage: python_recovered_runtime_probe.py <repo-root> <recovered-library>")

    repo_root = Path(sys.argv[1]).resolve()
    recovered_library = Path(sys.argv[2]).resolve()
    if not repo_root.exists():
        fail(f"repository root not found: {repo_root}")
    if not recovered_library.exists():
        fail(f"recovered library not found: {recovered_library}")

    with tempfile.TemporaryDirectory(prefix="bpx-sdk-python-probe-") as temp_dir:
        temp_root = Path(temp_dir)
        install_root = temp_root / "site"
        install_package(repo_root, recovered_library, install_root)

        os.chdir(temp_root)
        sys.path.insert(0, str(install_root))

        import bpx_sdk

        state = bpx_sdk.RequestRobotState()
        queried_version = state.queryRobotVersion()
        if queried_version is None or len(queried_version) != 6:
            fail("queryRobotVersion failed")
        if state.getRobotVersion() is not None:
            fail("getRobotVersion should be None before connect")
        if not state.connect():
            fail("RequestRobotState connect failed")
        if state.getRobotVersion() != queried_version:
            fail("connected RequestRobotState version mismatch")

        joint_pos = state.getJointPosition()
        imu_quat = state.getImuQuat()
        leg_odom = state.getLegOdom()
        current_state = state.getCurrentMotionState()
        if joint_pos is None or not close_enough(joint_pos[0], 0.0):
            fail("connected joint position not available")
        if imu_quat is None or not close_enough(imu_quat[3], 1.0):
            fail("connected imu quat not available")
        if leg_odom is None or not close_enough(leg_odom["orientation"][3], 1.0):
            fail("connected leg odom not available")
        if current_state != bpx_sdk.MotionState.Passive:
            fail("connected motion state mismatch")

        motion = bpx_sdk.MotionLevelControl()
        if not motion.connect():
            fail("MotionLevelControl connect failed")
        motion.setVelocityControlFlag(True)
        motion.setBound()
        motion.setVelocity(0.2, 0.1, 0.3)

        gait = motion.getCurrentGait()
        sub_gait = motion.getSubGait()
        max_velocity = motion.getMaxVelocity()
        if gait != bpx_sdk.MotionGait.WalkPhase:
            fail("motion gait mismatch")
        if sub_gait != 1:
            fail("motion sub-gait mismatch")
        if motion.getCurrentMotionState() != bpx_sdk.MotionState.Motion:
            fail("motion state mismatch")
        if (
            max_velocity is None
            or not close_enough(max_velocity[0], 1.5)
            or not close_enough(max_velocity[1], 1.0)
            or not close_enough(max_velocity[2], 2.0)
        ):
            fail("motion max velocity mismatch")
        if not motion.setDamping() or motion.getCurrentMotionState() != bpx_sdk.MotionState.Passive:
            fail("motion damping feedback mismatch")

        joint = bpx_sdk.JointLevelControl()
        if not joint.connect():
            fail("JointLevelControl connect failed")
        pos = [0.0] * 12
        vel = [0.0] * 12
        tff = [0.0] * 12
        pos[0] = 1.25
        vel[0] = -0.75
        tff[0] = 0.5
        if not joint.setJointCommand([0.0] * 12, pos, [0.0] * 12, vel, tff):
            fail("setJointCommand failed")

        high_rate_pos = joint.getJointPositionHighRate()
        high_rate_vel = joint.getJointVelocityHighRate()
        high_rate_tau = joint.getJointTorqueHighRate()
        high_rate_timestamp = joint.getJointStateTimestampHighRate()
        high_rate_seq = joint.getJointStateSeqHighRate()
        if high_rate_pos is None or not close_enough(high_rate_pos[0], pos[0]):
            fail("joint high-rate position mismatch")
        if high_rate_vel is None or not close_enough(high_rate_vel[0], vel[0]):
            fail("joint high-rate velocity mismatch")
        if high_rate_tau is None or not close_enough(high_rate_tau[0], tff[0]):
            fail("joint high-rate torque mismatch")
        if high_rate_timestamp is None or high_rate_timestamp <= 0.0:
            fail("joint high-rate timestamp missing")
        if high_rate_seq is None or high_rate_seq == 0:
            fail("joint high-rate sequence missing")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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
    script = """
import json
import bpx_sdk

def emit(name, value):
    result[name] = value

result = {}
state = bpx_sdk.RequestRobotState()
emit("request.getRobotVersion.beforeConnect", state.getRobotVersion())
emit("request.getJointPosition.beforeConnect", state.getJointPosition())
motion = bpx_sdk.MotionLevelControl()
motion.setWalk()
motion.setBound()
motion.setVelocityControlFlag(True)
motion.setVelocity(0.1, 0.2, 0.3)
motion.setStandUp()
motion.setDamping()
emit("motion.getCurrentMotionState.afterCommands", motion.getCurrentMotionState())
emit("motion.getCurrentGait.afterCommands", motion.getCurrentGait())
emit("motion.getSubGait.afterCommands", motion.getSubGait())
joint = bpx_sdk.JointLevelControl()
zeros = [0.0] * 12
joint.setJointCommand(zeros, zeros, zeros, zeros, zeros)
joint.setJointPosition(zeros)
joint.setJointVelocity(zeros)
joint.setJointTorqueFeedForward(zeros)
joint.setZeroJointCommand()
emit("joint.getJointPositionHighRate.afterCommands", joint.getJointPositionHighRate())
emit("joint.getJointVelocityHighRate.afterCommands", joint.getJointVelocityHighRate())
emit("joint.getJointTorqueHighRate.afterCommands", joint.getJointTorqueHighRate())
emit("joint.getJointStateTimestampHighRate.afterCommands", joint.getJointStateTimestampHighRate())
emit("joint.getJointStateSeqHighRate.afterCommands", joint.getJointStateSeqHighRate())
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
        fail("usage: python_compare_api_assumptions.py <repo-root> <shipped-library> <recovered-library>")

    repo_root = Path(sys.argv[1]).resolve()
    shipped_library = Path(sys.argv[2]).resolve()
    recovered_library = Path(sys.argv[3]).resolve()
    if not repo_root.exists():
        fail(f"repository root not found: {repo_root}")
    if not shipped_library.exists():
        fail(f"shipped library not found: {shipped_library}")
    if not recovered_library.exists():
        fail(f"recovered library not found: {recovered_library}")

    with tempfile.TemporaryDirectory(prefix="bpx-sdk-python-compare-") as temp_dir:
        temp_root = Path(temp_dir)
        shipped_root = temp_root / "shipped"
        recovered_root = temp_root / "recovered"
        install_package(repo_root, shipped_root, None)
        install_package(repo_root, recovered_root, recovered_library)

        precompiled = probe_behavior(shipped_root)
        recovered = probe_behavior(recovered_root)
        if precompiled != recovered:
            fail(
                "Recovered and precompiled Python probes diverged.\n"
                f"--- recovered ---\n{json.dumps(recovered, indent=2, sort_keys=True)}\n"
                f"--- precompiled ---\n{json.dumps(precompiled, indent=2, sort_keys=True)}"
            )

        shutil.rmtree(repo_root / "build", ignore_errors=True)
        shutil.rmtree(repo_root / "bpx_sdk_open.egg-info", ignore_errors=True)
        gc.collect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

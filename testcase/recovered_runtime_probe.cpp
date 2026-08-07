#include "joint_level_control.h"
#include "motion_level_control.h"
#include "request_robot_state.h"

#include "bpx_sdk_version.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

bool closeEnough(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1e-5f;
}

int fail(const char* message) {
    std::cerr << message << '\n';
    return 1;
}

}  // namespace

int main() {
    bpx_sdk::RequestRobotState state;
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;
    uint32_t commit = 0;
    uint32_t build_date = 0;
    uint32_t build_time = 0;
    if (!state.queryRobotVersion(&major, &minor, &patch, &commit, &build_date, &build_time)) {
        return fail("queryRobotVersion failed");
    }
    if (major != BPX_SDK_VERSION_MAJOR || minor != BPX_SDK_VERSION_MINOR || patch != BPX_SDK_VERSION_PATCH) {
        return fail("queryRobotVersion returned unexpected version");
    }
    if (state.getRobotVersion(&major, &minor, &patch, &commit, &build_date, &build_time)) {
        return fail("getRobotVersion should be false before connect");
    }
    if (!state.connect()) {
        return fail("RequestRobotState connect failed");
    }
    if (!state.getRobotVersion(&major, &minor, &patch, &commit, &build_date, &build_time)) {
        return fail("getRobotVersion should be true after connect");
    }

    float joint_pos[12] = {};
    float imu_quat[4] = {};
    uint8_t current_state = 0;
    if (!state.getJointPosition(joint_pos) || !closeEnough(joint_pos[0], 0.0f)) {
        return fail("connected joint position not available");
    }
    if (!state.getImuQuat(imu_quat) || !closeEnough(imu_quat[3], 1.0f)) {
        return fail("connected imu quat not available");
    }
    if (!state.getCurrentMotionState(&current_state) ||
        current_state != static_cast<uint8_t>(bpx_sdk::MotionState::Passive)) {
        return fail("connected motion state mismatch");
    }

    bpx_sdk::MotionLevelControl motion;
    if (!motion.connect()) {
        return fail("MotionLevelControl connect failed");
    }
    motion.setVelocityControlFlag(true);
    motion.setBound();
    motion.setVelocity(0.2f, 0.1f, 0.3f);

    uint8_t gait = 0;
    uint8_t sub_gait = 0;
    float max_velocity[3] = {};
    if (!motion.getCurrentGait(&gait) ||
        gait != static_cast<uint8_t>(bpx_sdk::MotionGait::WalkPhase)) {
        return fail("motion gait mismatch");
    }
    if (!motion.getSubGait(&sub_gait) || static_cast<int8_t>(sub_gait) != 1) {
        return fail("motion sub-gait mismatch");
    }
    if (!motion.getCurrentMotionState(&current_state) ||
        current_state != static_cast<uint8_t>(bpx_sdk::MotionState::Motion)) {
        return fail("motion state mismatch");
    }
    if (!motion.getMaxVelocity(max_velocity) ||
        !closeEnough(max_velocity[0], 1.5f) ||
        !closeEnough(max_velocity[1], 1.0f) ||
        !closeEnough(max_velocity[2], 2.0f)) {
        return fail("motion max velocity mismatch");
    }
    if (!motion.setDamping() ||
        !motion.getCurrentMotionState(&current_state) ||
        current_state != static_cast<uint8_t>(bpx_sdk::MotionState::Passive)) {
        return fail("motion damping feedback mismatch");
    }

    bpx_sdk::JointLevelControl joint;
    if (!joint.connect()) {
        return fail("JointLevelControl connect failed");
    }
    std::array<float, 12> pos{};
    std::array<float, 12> vel{};
    std::array<float, 12> tff{};
    pos[0] = 1.25f;
    vel[0] = -0.75f;
    tff[0] = 0.5f;
    if (!joint.setJointCommand({}, pos, {}, vel, tff)) {
        return fail("setJointCommand failed");
    }

    float high_rate_pos[12] = {};
    float high_rate_vel[12] = {};
    float high_rate_tau[12] = {};
    float high_rate_timestamp = 0.0f;
    uint32_t high_rate_seq = 0;
    if (!joint.getJointPositionHighRate(high_rate_pos) || !closeEnough(high_rate_pos[0], pos[0])) {
        return fail("joint high-rate position mismatch");
    }
    if (!joint.getJointVelocityHighRate(high_rate_vel) || !closeEnough(high_rate_vel[0], vel[0])) {
        return fail("joint high-rate velocity mismatch");
    }
    if (!joint.getJointTorqueHighRate(high_rate_tau) || !closeEnough(high_rate_tau[0], tff[0])) {
        return fail("joint high-rate torque mismatch");
    }
    if (!joint.getJointStateTimestampHighRate(&high_rate_timestamp) || high_rate_timestamp <= 0.0f) {
        return fail("joint high-rate timestamp missing");
    }
    if (!joint.getJointStateSeqHighRate(&high_rate_seq) || high_rate_seq == 0) {
        return fail("joint high-rate sequence missing");
    }

    return 0;
}

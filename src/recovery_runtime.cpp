#include "recovery_runtime.h"

#include "bpx_sdk_version.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>

namespace bpx_sdk {

SubscribeStateReq::SubscribeStateReq() = default;

uint32_t nowMs() {
    using clock = std::chrono::steady_clock;
    const auto ticks = std::chrono::duration_cast<std::chrono::milliseconds>(
        clock::now().time_since_epoch());
    return static_cast<uint32_t>(ticks.count() & 0xffffffffu);
}

uint32_t EncodeVersion(uint16_t major, uint16_t minor, uint16_t patch) {
    return (static_cast<uint32_t>(major) << 24) |
           (static_cast<uint32_t>(minor) << 16) |
           static_cast<uint32_t>(patch);
}

uint32_t EncodeCommitHash(const char* commit_hash) {
    if (!commit_hash) {
        return 0;
    }
    uint32_t value = 0;
    for (const char* cursor = commit_hash; *cursor; ++cursor) {
        if (!std::isxdigit(static_cast<unsigned char>(*cursor))) {
            continue;
        }
        value <<= 4;
        if (*cursor >= '0' && *cursor <= '9') {
            value |= static_cast<uint32_t>(*cursor - '0');
        } else {
            value |= static_cast<uint32_t>(std::tolower(static_cast<unsigned char>(*cursor)) - 'a' + 10);
        }
    }
    return value;
}

RobotVersionInfo currentSdkVersion() {
    RobotVersionInfo info;
    info.major = BPX_SDK_VERSION_MAJOR;
    info.minor = BPX_SDK_VERSION_MINOR;
    info.patch = BPX_SDK_VERSION_PATCH;
    info.commit = EncodeCommitHash(BPX_SDK_GIT_COMMIT_HASH);
    return info;
}

std::array<float, 3> gaitVelocityLimit(MotionGait gait) {
    switch (gait) {
        case MotionGait::Walk:
            return {1.5f, 0.5f, 2.0f};
        case MotionGait::Bipedal:
            return {0.8f, 0.5f, 1.5f};
        case MotionGait::Flip:
            return {0.0f, 0.0f, 0.0f};
        case MotionGait::WalkPhase:
            return {1.5f, 1.0f, 2.0f};
        case MotionGait::PoseTracking:
            return {0.0f, 0.0f, 0.0f};
        case MotionGait::Running:
            return {3.0f, 1.0f, 2.0f};
    }
    return {};
}

RobotStateSnapshot makeConnectedSnapshot() {
    RobotStateSnapshot snapshot;
    snapshot.max_velocity = gaitVelocityLimit(MotionGait::Walk);
    snapshot.leg_odom.orientation[3] = 1.0f;
    const uint32_t timestamp = nowMs();
    snapshot.joint_state_timestamp = timestamp;
    snapshot.imu_timestamp = timestamp;
    snapshot.odometry_timestamp = timestamp;
    snapshot.motion_state_timestamp = timestamp;
    snapshot.battery_timestamp = timestamp;
    return snapshot;
}

void applyGaitSelection(RobotStateSnapshot* snapshot, MotionGait gait, int8_t sub_gait) {
    if (!snapshot) {
        return;
    }
    snapshot->last_gait = snapshot->current_gait;
    snapshot->current_gait = static_cast<uint8_t>(gait);
    snapshot->sub_gait = static_cast<uint8_t>(sub_gait);
    snapshot->max_velocity = gaitVelocityLimit(gait);
    snapshot->motion_state_timestamp = nowMs();
}

void applyMotionState(RobotStateSnapshot* snapshot, MotionState state) {
    if (!snapshot) {
        return;
    }
    snapshot->last_motion_state = snapshot->current_motion_state;
    snapshot->current_motion_state = static_cast<uint8_t>(state);
    snapshot->motion_state_timestamp = nowMs();
}

}  // namespace bpx_sdk

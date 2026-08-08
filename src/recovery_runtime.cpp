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

int parseTwoDigits(const char tens, const char ones) {
    if (tens < '0' || tens > '9' || ones < '0' || ones > '9') {
        return 0;
    }
    return (tens - '0') * 10 + (ones - '0');
}

int parseDay(const char* date) {
    if (!date) {
        return 0;
    }
    if (date[4] == ' ') {
        return parseTwoDigits('0', date[5]);
    }
    return parseTwoDigits(date[4], date[5]);
}

int parseYear(const char* date) {
    if (!date) {
        return 0;
    }
    return (date[7] - '0') * 1000 +
           (date[8] - '0') * 100 +
           (date[9] - '0') * 10 +
           (date[10] - '0');
}

int parseMonth(const char* date) {
    if (!date) {
        return 0;
    }
    if (std::strncmp(date, "Jan", 3) == 0) return 1;
    if (std::strncmp(date, "Feb", 3) == 0) return 2;
    if (std::strncmp(date, "Mar", 3) == 0) return 3;
    if (std::strncmp(date, "Apr", 3) == 0) return 4;
    if (std::strncmp(date, "May", 3) == 0) return 5;
    if (std::strncmp(date, "Jun", 3) == 0) return 6;
    if (std::strncmp(date, "Jul", 3) == 0) return 7;
    if (std::strncmp(date, "Aug", 3) == 0) return 8;
    if (std::strncmp(date, "Sep", 3) == 0) return 9;
    if (std::strncmp(date, "Oct", 3) == 0) return 10;
    if (std::strncmp(date, "Nov", 3) == 0) return 11;
    if (std::strncmp(date, "Dec", 3) == 0) return 12;
    return 0;
}

uint32_t compileBuildDate() {
    const char* date = __DATE__;
    return static_cast<uint32_t>(parseYear(date) * 10000 +
                                 parseMonth(date) * 100 +
                                 parseDay(date));
}

uint32_t compileBuildTime() {
    const char* time = __TIME__;
    return static_cast<uint32_t>(parseTwoDigits(time[0], time[1]) * 10000 +
                                 parseTwoDigits(time[3], time[4]) * 100 +
                                 parseTwoDigits(time[6], time[7]));
}

RobotVersionInfo currentSdkVersion() {
    RobotVersionInfo info;
    info.major = BPX_SDK_VERSION_MAJOR;
    info.minor = BPX_SDK_VERSION_MINOR;
    info.patch = BPX_SDK_VERSION_PATCH;
    info.commit = EncodeCommitHash(BPX_SDK_GIT_COMMIT_HASH);
    info.build_date = compileBuildDate();
    info.build_time = compileBuildTime();
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

#ifndef BPX_SDK_RECOVERY_RUNTIME_H_
#define BPX_SDK_RECOVERY_RUNTIME_H_

#include "motion_types.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace bpx_sdk {

struct RobotVersionInfo {
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;
    uint32_t commit = 0;
    uint32_t build_date = 0;
    uint32_t build_time = 0;
};

struct SubscribeStateReq {
    SubscribeStateReq();

    uint16_t session_id = 0;
    uint16_t robot_state_upload_port = 0;
    uint16_t joint_state_upload_port = 0;
    uint16_t reserved = 0;
    uint16_t robot_state_upload_rate_hz = 0;
    uint8_t host_server_mode = 0;
    uint8_t reserved_padding = 0;
    uint32_t request_timestamp_ms = 0;
    uint32_t reserved_word0 = 0;
    uint32_t reserved_word1 = 0;
};

struct SubscribeStateResp {
    std::array<uint8_t, 32> raw{};

    uint8_t responseType() const { return raw[0]; }
    uint8_t statusCode() const { return raw[1]; }
    uint16_t reserved() const {
        return static_cast<uint16_t>(raw[2]) |
               (static_cast<uint16_t>(raw[3]) << 8);
    }
    uint32_t payloadWord(std::size_t index) const {
        if (index >= 7) {
            return 0;
        }
        const std::size_t base = 4 + index * 4;
        return static_cast<uint32_t>(raw[base]) |
               (static_cast<uint32_t>(raw[base + 1]) << 8) |
               (static_cast<uint32_t>(raw[base + 2]) << 16) |
               (static_cast<uint32_t>(raw[base + 3]) << 24);
    }
    bool accepted() const { return responseType() != 0 || statusCode() != 0; }
};

struct JointCommandPacket {
    std::array<float, 12> kp{};
    std::array<float, 12> pos{};
    std::array<float, 12> kd{};
    std::array<float, 12> vel{};
    std::array<float, 12> tff{};
};

struct JointStatePacket {
    std::array<float, 12> joint_position{};
    std::array<float, 12> joint_velocity{};
    std::array<float, 12> joint_torque{};
    std::array<float, 3> imu_rpy{};
    std::array<float, 4> imu_quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> imu_acc{};
    std::array<float, 3> imu_omega{};
    float timestamp_ms = 0.0f;
    uint32_t seq = 0;
};

struct ClientUploadPacketHead {
    uint32_t seq = 0;
    uint32_t timestamp_ms = 0;
    uint16_t payload_size = 0;
    uint16_t payload_type = 0;
};

struct ClientUploadData1Hz {
    uint8_t battery_level = 0;
    uint8_t reserved[3]{};
    float battery_current = 0.0f;
    std::array<int8_t, 12> motor_temperature{};
    std::array<int8_t, 12> driver_temperature{};
};

struct ClientUploadData10Hz {
    uint8_t current_motion_state = 0;
    uint8_t current_gait = 0;
    uint8_t last_motion_state = 0;
    uint8_t last_gait = 0;
    int8_t sub_gait = 0;
    uint8_t reserved[3]{};
    std::array<float, 3> max_velocity{};
};

struct ClientUploadData50Hz {
    LegOdom leg_odom{};
};

struct ClientUploadData200Hz {
    std::array<float, 3> imu_rpy{};
    std::array<float, 4> imu_quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> imu_acc{};
    std::array<float, 3> imu_omega{};
};

struct ClientUploadData1000Hz {
    std::array<float, 12> joint_position{};
    std::array<float, 12> joint_velocity{};
    std::array<float, 12> joint_torque{};
};

struct RobotStateSnapshot {
    std::array<float, 12> joint_position{};
    std::array<float, 12> joint_velocity{};
    std::array<float, 12> joint_torque{};
    std::array<float, 3> imu_rpy{};
    std::array<float, 4> imu_quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> imu_acc{};
    std::array<float, 3> imu_omega{};
    LegOdom leg_odom{};
    std::array<float, 12> motor_temperature{};
    std::array<float, 12> driver_temperature{};
    std::array<float, 3> max_velocity{};
    uint8_t battery_level = 100;
    float battery_current = 0.0f;
    uint8_t current_motion_state = static_cast<uint8_t>(MotionState::Passive);
    uint8_t current_gait = static_cast<uint8_t>(MotionGait::Walk);
    uint8_t last_motion_state = static_cast<uint8_t>(MotionState::Passive);
    uint8_t last_gait = static_cast<uint8_t>(MotionGait::Walk);
    uint8_t sub_gait = 0;
    uint32_t joint_state_timestamp = 0;
    uint32_t imu_timestamp = 0;
    uint32_t odometry_timestamp = 0;
    uint32_t motion_state_timestamp = 0;
    uint32_t battery_timestamp = 0;
};

uint32_t nowMs();
uint32_t EncodeVersion(uint16_t major, uint16_t minor, uint16_t patch);
uint32_t EncodeCommitHash(const char* commit_hash);
RobotVersionInfo currentSdkVersion();
std::array<float, 3> gaitVelocityLimit(MotionGait gait);
RobotStateSnapshot makeConnectedSnapshot();
void applyGaitSelection(RobotStateSnapshot* snapshot, MotionGait gait, int8_t sub_gait);
void applyMotionState(RobotStateSnapshot* snapshot, MotionState state);

}  // namespace bpx_sdk

static_assert(sizeof(bpx_sdk::SubscribeStateReq) == 24,
              "SubscribeStateReq must match the recovered 24-byte wire layout");
static_assert(sizeof(bpx_sdk::SubscribeStateResp) == 32,
              "SubscribeStateResp must match the recovered 32-byte wire layout");

#endif  // BPX_SDK_RECOVERY_RUNTIME_H_

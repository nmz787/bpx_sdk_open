#ifndef BPX_SDK_RECOVERY_RUNTIME_H_
#define BPX_SDK_RECOVERY_RUNTIME_H_

#include "motion_types.h"

#include <array>
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
    uint16_t robot_state_upload_rate_hz = 0;
    uint8_t host_server_mode = 0;
};

struct SubscribeStateResp {
    bool accepted = true;
    uint16_t session_id = 0;
    uint16_t robot_state_upload_port = 0;
    uint16_t joint_state_upload_port = 0;
    uint16_t robot_state_upload_rate_hz = 0;
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

struct ClientUploadData1Hz {};
struct ClientUploadData10Hz {};
struct ClientUploadData50Hz {};
struct ClientUploadData200Hz {};
struct ClientUploadData1000Hz {};

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

#endif  // BPX_SDK_RECOVERY_RUNTIME_H_

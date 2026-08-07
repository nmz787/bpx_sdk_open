#include "request_robot_state.h"

#include "bpx_sdk_version.h"
#include "robot_state_udp_receiver.h"
#include "tcp_subscribe_client.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <optional>

namespace bpx_sdk {
namespace {

template <size_t N>
bool copyFloatArray(const std::array<float, N>& src, float* dst) {
    if (!dst) {
        return false;
    }
    std::copy(src.begin(), src.end(), dst);
    return true;
}

template <size_t N>
std::optional<std::array<float, N>> toOptionalArray(const std::array<float, N>& src) {
    return src;
}

bool decodeMotionState(uint8_t raw, MotionState* state) {
    if (!state) {
        return false;
    }
    switch (raw) {
        case static_cast<uint8_t>(MotionState::LyingDown):
        case static_cast<uint8_t>(MotionState::StandingUp):
        case static_cast<uint8_t>(MotionState::Passive):
        case static_cast<uint8_t>(MotionState::SitDown):
        case static_cast<uint8_t>(MotionState::Motion):
            *state = static_cast<MotionState>(raw);
            return true;
        default:
            return false;
    }
}

bool decodeMotionGait(uint8_t raw, MotionGait* gait) {
    if (!gait) {
        return false;
    }
    switch (raw) {
        case static_cast<uint8_t>(MotionGait::Walk):
        case static_cast<uint8_t>(MotionGait::Bipedal):
        case static_cast<uint8_t>(MotionGait::Flip):
        case static_cast<uint8_t>(MotionGait::WalkPhase):
        case static_cast<uint8_t>(MotionGait::PoseTracking):
        case static_cast<uint8_t>(MotionGait::Running):
            *gait = static_cast<MotionGait>(raw);
            return true;
        default:
            return false;
    }
}

void writeVersionOutputs(const RobotVersionInfo& version,
                         uint16_t* major,
                         uint16_t* minor,
                         uint16_t* patch,
                         uint32_t* commit,
                         uint32_t* build_date,
                         uint32_t* build_time) {
    if (major) *major = version.major;
    if (minor) *minor = version.minor;
    if (patch) *patch = version.patch;
    if (commit) *commit = version.commit;
    if (build_date) *build_date = version.build_date;
    if (build_time) *build_time = version.build_time;
}

}  // namespace

class RequestRobotState::Impl {
public:
    bool connected = false;
    uint16_t robot_state_upload_port = DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT;
    uint16_t joint_state_upload_port = DEFAULT_CLIENT_JOINT_STATE_UDP_PORT;
    uint16_t robot_state_upload_rate_hz = 100;
    uint16_t tcp_local_port = 0;
    uint16_t session_id = 0;
    uint16_t robot_version_major = BPX_SDK_VERSION_MAJOR;
    uint16_t robot_version_minor = BPX_SDK_VERSION_MINOR;
    uint16_t robot_version_patch = BPX_SDK_VERSION_PATCH;
    uint32_t robot_version_commit = 0;
    uint32_t robot_version_build_date = 0;
    uint32_t robot_version_build_time = 0;
    bool has_robot_version = false;
    char robot_ip[64] = {};
    std::unique_ptr<TcpSubscribeClient> tcp_client;
    std::unique_ptr<RobotStateUdpReceiver> state_receiver;

    std::optional<std::array<float, 12>> joint_position;
    std::optional<std::array<float, 12>> joint_velocity;
    std::optional<std::array<float, 12>> joint_torque;
    std::optional<std::array<float, 3>> imu_rpy;
    std::optional<std::array<float, 4>> imu_quat;
    std::optional<std::array<float, 3>> imu_acc;
    std::optional<std::array<float, 3>> imu_omega;
    std::optional<LegOdom> leg_odom;
    std::optional<std::array<float, 12>> motor_temperature;
    std::optional<std::array<float, 12>> driver_temperature;
    std::optional<std::array<float, 3>> max_velocity;
    std::optional<uint8_t> battery_level;
    std::optional<float> battery_current;
    std::optional<uint8_t> current_motion_state;
    std::optional<uint8_t> current_gait;
    std::optional<uint8_t> last_motion_state;
    std::optional<uint8_t> last_gait;
    std::optional<uint8_t> sub_gait;
    std::optional<uint32_t> joint_state_timestamp;
    std::optional<uint32_t> imu_timestamp;
    std::optional<uint32_t> odometry_timestamp;
    std::optional<uint32_t> motion_state_timestamp;
    std::optional<uint32_t> battery_timestamp;

    void applySnapshot(const RobotStateSnapshot& snapshot) {
        joint_position = snapshot.joint_position;
        joint_velocity = snapshot.joint_velocity;
        joint_torque = snapshot.joint_torque;
        imu_rpy = snapshot.imu_rpy;
        imu_quat = snapshot.imu_quat;
        imu_acc = snapshot.imu_acc;
        imu_omega = snapshot.imu_omega;
        leg_odom = snapshot.leg_odom;
        motor_temperature = snapshot.motor_temperature;
        driver_temperature = snapshot.driver_temperature;
        max_velocity = snapshot.max_velocity;
        battery_level = snapshot.battery_level;
        battery_current = snapshot.battery_current;
        current_motion_state = snapshot.current_motion_state;
        current_gait = snapshot.current_gait;
        last_motion_state = snapshot.last_motion_state;
        last_gait = snapshot.last_gait;
        sub_gait = snapshot.sub_gait;
        joint_state_timestamp = snapshot.joint_state_timestamp;
        imu_timestamp = snapshot.imu_timestamp;
        odometry_timestamp = snapshot.odometry_timestamp;
        motion_state_timestamp = snapshot.motion_state_timestamp;
        battery_timestamp = snapshot.battery_timestamp;
    }
};

RequestRobotState::RequestRobotState()
    : impl_(std::make_unique<Impl>()) {
    std::snprintf(impl_->robot_ip, sizeof(impl_->robot_ip), "%s", DEFAULT_SERVER_IP);
}

RequestRobotState::~RequestRobotState() = default;

bool RequestRobotState::connect() {
    if (!impl_->state_receiver) {
        impl_->state_receiver = std::make_unique<RobotStateUdpReceiver>(impl_->robot_state_upload_port);
    }
    impl_->state_receiver->setListenPort(impl_->robot_state_upload_port);
    if (!impl_->state_receiver->connect()) {
        return false;
    }

    if (!impl_->tcp_client) {
        impl_->tcp_client = std::make_unique<TcpSubscribeClient>();
    }
    impl_->tcp_client->attachReceiver(impl_->state_receiver.get());
    impl_->tcp_client->setSessionId(impl_->session_id);
    impl_->tcp_client->setTcpLocalPort(impl_->tcp_local_port);
    impl_->tcp_client->setRobotStateUploadPort(impl_->robot_state_upload_port);
    impl_->tcp_client->setJointStateUploadPort(impl_->joint_state_upload_port);
    impl_->tcp_client->setRobotStateUploadRate(impl_->robot_state_upload_rate_hz);
    impl_->tcp_client->setHostServerMode(hostServerMode());
    if (!impl_->tcp_client->startStateQuery()) {
        return false;
    }

    RobotVersionInfo version{};
    if (impl_->tcp_client->queryRobotVersion(&version)) {
        impl_->robot_version_major = version.major;
        impl_->robot_version_minor = version.minor;
        impl_->robot_version_patch = version.patch;
        impl_->robot_version_commit = version.commit;
        impl_->robot_version_build_date = version.build_date;
        impl_->robot_version_build_time = version.build_time;
        impl_->has_robot_version = true;
    }

    RobotStateSnapshot snapshot;
    if (impl_->state_receiver->getLatestState(&snapshot)) {
        impl_->applySnapshot(snapshot);
    }

    impl_->connected = true;
    return true;
}

void RequestRobotState::disconnect() {
    if (impl_->tcp_client) {
        impl_->tcp_client->disconnect();
    }
    if (impl_->state_receiver) {
        impl_->state_receiver->disconnect();
    }
    impl_->connected = false;
}

void RequestRobotState::setRobotStateUploadPort(uint16_t port) { impl_->robot_state_upload_port = port; }
void RequestRobotState::setJointStateUploadPort(uint16_t port) { impl_->joint_state_upload_port = port; }
void RequestRobotState::setRobotStateUploadRate(uint16_t rate_hz) { impl_->robot_state_upload_rate_hz = rate_hz; }
void RequestRobotState::setTcpLocalPort(uint16_t port) { impl_->tcp_local_port = port; }
void RequestRobotState::setSessionId(uint16_t session_id) { impl_->session_id = session_id; }

void RequestRobotState::setRobotIp(const char* ip) {
    if (!ip || ip[0] == '\0') {
        return;
    }
    std::snprintf(impl_->robot_ip, sizeof(impl_->robot_ip), "%s", ip);
}

bool RequestRobotState::queryRobotVersion(uint16_t* major, uint16_t* minor, uint16_t* patch,
                                          uint32_t* commit, uint32_t* build_date,
                                          uint32_t* build_time) {
    TcpSubscribeClient tcp_client;
    tcp_client.setSessionId(impl_->session_id);
    tcp_client.setTcpLocalPort(impl_->tcp_local_port);
    tcp_client.setRobotStateUploadPort(impl_->robot_state_upload_port);
    tcp_client.setJointStateUploadPort(impl_->joint_state_upload_port);
    tcp_client.setRobotStateUploadRate(impl_->robot_state_upload_rate_hz);
    tcp_client.setHostServerMode(hostServerMode());
    RobotVersionInfo version{};
    if (!tcp_client.queryRobotVersion(&version)) {
        return false;
    }
    writeVersionOutputs(version, major, minor, patch, commit, build_date, build_time);
    return true;
}

bool RequestRobotState::getRobotVersion(uint16_t* major, uint16_t* minor, uint16_t* patch,
                                        uint32_t* commit, uint32_t* build_date,
                                        uint32_t* build_time) const {
    if (!impl_->has_robot_version) {
        return false;
    }
    const RobotVersionInfo version{
        impl_->robot_version_major,
        impl_->robot_version_minor,
        impl_->robot_version_patch,
        impl_->robot_version_commit,
        impl_->robot_version_build_date,
        impl_->robot_version_build_time,
    };
    writeVersionOutputs(version, major, minor, patch, commit, build_date, build_time);
    return true;
}

bool RequestRobotState::getJointPosition(float joint_pos[12]) const { return impl_->joint_position && copyFloatArray(*impl_->joint_position, joint_pos); }
bool RequestRobotState::getJointVelocity(float joint_vel[12]) const { return impl_->joint_velocity && copyFloatArray(*impl_->joint_velocity, joint_vel); }
bool RequestRobotState::getJointTorque(float joint_tau[12]) const { return impl_->joint_torque && copyFloatArray(*impl_->joint_torque, joint_tau); }
bool RequestRobotState::getImuRpy(float rpy[3]) const { return impl_->imu_rpy && copyFloatArray(*impl_->imu_rpy, rpy); }
bool RequestRobotState::getImuQuat(float quat[4]) const { return impl_->imu_quat && copyFloatArray(*impl_->imu_quat, quat); }
bool RequestRobotState::getImuAcc(float acc[3]) const { return impl_->imu_acc && copyFloatArray(*impl_->imu_acc, acc); }
bool RequestRobotState::getImuOmega(float omega[3]) const { return impl_->imu_omega && copyFloatArray(*impl_->imu_omega, omega); }

bool RequestRobotState::getLegOdom(LegOdom* leg_odom) const {
    if (!leg_odom || !impl_->leg_odom) return false;
    *leg_odom = *impl_->leg_odom;
    return true;
}

bool RequestRobotState::getMotorTemperature(float motor_temperature[12]) const { return impl_->motor_temperature && copyFloatArray(*impl_->motor_temperature, motor_temperature); }
bool RequestRobotState::getDriverTemperature(float driver_temperature[12]) const { return impl_->driver_temperature && copyFloatArray(*impl_->driver_temperature, driver_temperature); }

bool RequestRobotState::getCurrentMotionState(uint8_t* current_state) const {
    if (!current_state || !impl_->current_motion_state) return false;
    *current_state = *impl_->current_motion_state;
    return true;
}

bool RequestRobotState::getCurrentGait(uint8_t* current_gait) const {
    if (!current_gait || !impl_->current_gait) return false;
    *current_gait = *impl_->current_gait;
    return true;
}

bool RequestRobotState::getLastMotionState(uint8_t* last_state) const {
    if (!last_state || !impl_->last_motion_state) return false;
    *last_state = *impl_->last_motion_state;
    return true;
}

bool RequestRobotState::getLastGait(uint8_t* last_gait) const {
    if (!last_gait || !impl_->last_gait) return false;
    *last_gait = *impl_->last_gait;
    return true;
}

bool RequestRobotState::getSubGait(uint8_t* sub_gait) const {
    if (!sub_gait || !impl_->sub_gait) return false;
    *sub_gait = *impl_->sub_gait;
    return true;
}

bool RequestRobotState::getCurrentMotionState(MotionState* current_state) const { return impl_->current_motion_state && decodeMotionState(*impl_->current_motion_state, current_state); }
bool RequestRobotState::getCurrentGait(MotionGait* current_gait) const { return impl_->current_gait && decodeMotionGait(*impl_->current_gait, current_gait); }
bool RequestRobotState::getLastMotionState(MotionState* last_state) const { return impl_->last_motion_state && decodeMotionState(*impl_->last_motion_state, last_state); }
bool RequestRobotState::getLastGait(MotionGait* last_gait) const { return impl_->last_gait && decodeMotionGait(*impl_->last_gait, last_gait); }
bool RequestRobotState::getMaxVelocity(float max_vel[3]) const { return impl_->max_velocity && copyFloatArray(*impl_->max_velocity, max_vel); }

bool RequestRobotState::getBatteryLevel(uint8_t* battery_level) const {
    if (!battery_level || !impl_->battery_level) return false;
    *battery_level = *impl_->battery_level;
    return true;
}

bool RequestRobotState::getBatteryCurrent(float* battery_current) const {
    if (!battery_current || !impl_->battery_current) return false;
    *battery_current = *impl_->battery_current;
    return true;
}

bool RequestRobotState::getJointStateTimestamp(uint32_t* time_ms) const {
    if (!time_ms || !impl_->joint_state_timestamp) return false;
    *time_ms = *impl_->joint_state_timestamp;
    return true;
}

bool RequestRobotState::getImuTimestamp(uint32_t* time_ms) const {
    if (!time_ms || !impl_->imu_timestamp) return false;
    *time_ms = *impl_->imu_timestamp;
    return true;
}

bool RequestRobotState::getOdometryTimestamp(uint32_t* time_ms) const {
    if (!time_ms || !impl_->odometry_timestamp) return false;
    *time_ms = *impl_->odometry_timestamp;
    return true;
}

bool RequestRobotState::getMotionStateTimestamp(uint32_t* time_ms) const {
    if (!time_ms || !impl_->motion_state_timestamp) return false;
    *time_ms = *impl_->motion_state_timestamp;
    return true;
}

bool RequestRobotState::getBatteryTimestamp(uint32_t* time_ms) const {
    if (!time_ms || !impl_->battery_timestamp) return false;
    *time_ms = *impl_->battery_timestamp;
    return true;
}

std::optional<std::array<float, 12>> RequestRobotState::getJointPositionArray() const { return impl_->joint_position; }
std::optional<std::array<float, 12>> RequestRobotState::getJointVelocityArray() const { return impl_->joint_velocity; }
std::optional<std::array<float, 12>> RequestRobotState::getJointTorqueArray() const { return impl_->joint_torque; }
std::optional<std::array<float, 3>> RequestRobotState::getImuRpyArray() const { return impl_->imu_rpy; }
std::optional<std::array<float, 4>> RequestRobotState::getImuQuatArray() const { return impl_->imu_quat; }
std::optional<std::array<float, 3>> RequestRobotState::getImuAccArray() const { return impl_->imu_acc; }
std::optional<std::array<float, 3>> RequestRobotState::getImuOmegaArray() const { return impl_->imu_omega; }
std::optional<LegOdom> RequestRobotState::getLegOdomValue() const { return impl_->leg_odom; }
std::optional<std::array<float, 12>> RequestRobotState::getMotorTemperatureArray() const { return impl_->motor_temperature; }
std::optional<std::array<float, 12>> RequestRobotState::getDriverTemperatureArray() const { return impl_->driver_temperature; }
std::optional<uint8_t> RequestRobotState::getCurrentMotionStateValue() const { return impl_->current_motion_state; }
std::optional<uint8_t> RequestRobotState::getCurrentGaitValue() const { return impl_->current_gait; }
std::optional<uint8_t> RequestRobotState::getLastMotionStateValue() const { return impl_->last_motion_state; }
std::optional<uint8_t> RequestRobotState::getLastGaitValue() const { return impl_->last_gait; }
std::optional<uint8_t> RequestRobotState::getSubGaitValue() const { return impl_->sub_gait; }

std::optional<MotionState> RequestRobotState::getCurrentMotionStateEnum() const {
    MotionState state{};
    return getCurrentMotionState(&state) ? std::optional<MotionState>(state) : std::nullopt;
}

std::optional<MotionGait> RequestRobotState::getCurrentGaitEnum() const {
    MotionGait gait{};
    return getCurrentGait(&gait) ? std::optional<MotionGait>(gait) : std::nullopt;
}

std::optional<MotionState> RequestRobotState::getLastMotionStateEnum() const {
    MotionState state{};
    return getLastMotionState(&state) ? std::optional<MotionState>(state) : std::nullopt;
}

std::optional<MotionGait> RequestRobotState::getLastGaitEnum() const {
    MotionGait gait{};
    return getLastGait(&gait) ? std::optional<MotionGait>(gait) : std::nullopt;
}

std::optional<std::array<float, 3>> RequestRobotState::getMaxVelocityArray() const { return impl_->max_velocity; }
std::optional<uint8_t> RequestRobotState::getBatteryLevelValue() const { return impl_->battery_level; }
std::optional<float> RequestRobotState::getBatteryCurrentValue() const { return impl_->battery_current; }
std::optional<uint32_t> RequestRobotState::getJointStateTimestampValue() const { return impl_->joint_state_timestamp; }
std::optional<uint32_t> RequestRobotState::getImuTimestampValue() const { return impl_->imu_timestamp; }
std::optional<uint32_t> RequestRobotState::getOdometryTimestampValue() const { return impl_->odometry_timestamp; }
std::optional<uint32_t> RequestRobotState::getMotionStateTimestampValue() const { return impl_->motion_state_timestamp; }
std::optional<uint32_t> RequestRobotState::getBatteryTimestampValue() const { return impl_->battery_timestamp; }

uint8_t RequestRobotState::hostServerMode() const { return 1; }
const char* RequestRobotState::robotIp() const { return impl_->robot_ip; }

void RequestRobotState::setCurrentMotionStateValue(MotionState state) {
    impl_->last_motion_state = impl_->current_motion_state;
    impl_->current_motion_state = static_cast<uint8_t>(state);
    impl_->motion_state_timestamp = nowMs();
}

void RequestRobotState::setMaxVelocityState(const std::array<float, 3>& max_velocity) {
    impl_->max_velocity = max_velocity;
}

bool RequestRobotState::isConnected() const { return impl_->connected; }
uint16_t RequestRobotState::robotStateUploadPort() const { return impl_->robot_state_upload_port; }
uint16_t RequestRobotState::jointStateUploadPort() const { return impl_->joint_state_upload_port; }
uint16_t RequestRobotState::robotStateUploadRate() const { return impl_->robot_state_upload_rate_hz; }
uint16_t RequestRobotState::tcpLocalPort() const { return impl_->tcp_local_port; }
uint16_t RequestRobotState::sessionId() const { return impl_->session_id; }

void RequestRobotState::setCurrentGaitState(MotionGait gait, int8_t sub_gait) {
    impl_->last_gait = impl_->current_gait;
    impl_->current_gait = static_cast<uint8_t>(gait);
    impl_->sub_gait = static_cast<uint8_t>(sub_gait);
    impl_->motion_state_timestamp = nowMs();
}

}  // namespace bpx_sdk

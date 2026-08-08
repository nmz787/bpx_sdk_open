#include "robot_state_udp_receiver.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bpx_sdk {
namespace {

constexpr uint16_t kPayloadType1000Hz = 0x1000;
constexpr uint16_t kPayloadType200Hz = 0x0200;
constexpr uint16_t kPayloadType50Hz = 0x0050;
constexpr uint16_t kPayloadType10Hz = 0x0010;
constexpr uint16_t kPayloadType1Hz = 0x0001;
constexpr float kTemperatureBaseCelsius = 60.0f;

template <size_t N>
bool copyArray(const std::array<float, N>& source, float* target) {
    if (!target) {
        return false;
    }
    std::copy(source.begin(), source.end(), target);
    return true;
}

template <typename T>
bool copyScalar(T value, T* target) {
    if (!target) {
        return false;
    }
    *target = value;
    return true;
}

template <typename T>
bool decodePayload(const unsigned char* data, unsigned long payload_size, T* payload) {
    if (!data || !payload || payload_size != sizeof(T)) {
        return false;
    }
    std::memcpy(payload, data, sizeof(T));
    return true;
}

void applyPayload(const ClientUploadData1000Hz& payload,
                  const ClientUploadPacketHead& head,
                  RobotStateSnapshot* snapshot) {
    snapshot->joint_position = payload.joint_position;
    snapshot->joint_velocity = payload.joint_velocity;
    snapshot->joint_torque = payload.joint_torque;
    snapshot->joint_state_timestamp = head.timestamp_ms;
}

void applyPayload(const ClientUploadData200Hz& payload,
                  const ClientUploadPacketHead& head,
                  RobotStateSnapshot* snapshot) {
    snapshot->imu_rpy = payload.imu_rpy;
    snapshot->imu_quat = payload.imu_quat;
    snapshot->imu_acc = payload.imu_acc;
    snapshot->imu_omega = payload.imu_omega;
    snapshot->imu_timestamp = head.timestamp_ms;
}

void applyPayload(const ClientUploadData50Hz& payload,
                  const ClientUploadPacketHead& head,
                  RobotStateSnapshot* snapshot) {
    snapshot->leg_odom = payload.leg_odom;
    snapshot->odometry_timestamp = head.timestamp_ms;
}

void applyPayload(const ClientUploadData10Hz& payload,
                  const ClientUploadPacketHead& head,
                  RobotStateSnapshot* snapshot) {
    snapshot->current_motion_state = payload.current_motion_state;
    snapshot->current_gait = payload.current_gait;
    snapshot->last_motion_state = payload.last_motion_state;
    snapshot->last_gait = payload.last_gait;
    snapshot->sub_gait = static_cast<uint8_t>(payload.sub_gait);
    snapshot->max_velocity = payload.max_velocity;
    snapshot->motion_state_timestamp = head.timestamp_ms;
}

void applyPayload(const ClientUploadData1Hz& payload,
                  const ClientUploadPacketHead& head,
                  RobotStateSnapshot* snapshot) {
    snapshot->battery_level = payload.battery_level;
    snapshot->battery_current = payload.battery_current;
    for (size_t i = 0; i < payload.motor_temperature.size(); ++i) {
        snapshot->motor_temperature[i] =
            kTemperatureBaseCelsius + static_cast<float>(payload.motor_temperature[i]);
        snapshot->driver_temperature[i] =
            kTemperatureBaseCelsius + static_cast<float>(payload.driver_temperature[i]);
    }
    snapshot->battery_timestamp = head.timestamp_ms;
}

}  // namespace

RobotStateUdpReceiver::RobotStateUdpReceiver(uint16_t listen_port)
    : listen_port_(listen_port) {}

RobotStateUdpReceiver::~RobotStateUdpReceiver() = default;

void RobotStateUdpReceiver::disconnect() {
    running_ = false;
    closeSocket();
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

bool RobotStateUdpReceiver::openSocket() {
    if (socket_open_) {
        return true;
    }
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }
    const int enabled = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    sockaddr_in address{};
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(listen_port_);
    if (bind(socket_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    socket_open_ = true;
    return true;
}

void RobotStateUdpReceiver::closeSocket() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    socket_open_ = false;
}

bool RobotStateUdpReceiver::parsePacket(const unsigned char* data, unsigned long size, bool) {
    if (!data || size < sizeof(ClientUploadPacketHead)) {
        return false;
    }

    ClientUploadPacketHead head;
    std::memcpy(&head, data, sizeof(head));
    if (size - sizeof(head) < head.payload_size) {
        return false;
    }

    const unsigned char* payload_data = data + sizeof(head);
    RobotStateSnapshot snapshot;
    if (!getLatestState(&snapshot)) {
        snapshot = makeConnectedSnapshot();
    }

    switch (head.payload_type) {
        case kPayloadType1000Hz: {
            ClientUploadData1000Hz payload;
            if (!decodePayload(payload_data, head.payload_size, &payload)) {
                return false;
            }
            applyPayload(payload, head, &snapshot);
            break;
        }
        case kPayloadType200Hz: {
            ClientUploadData200Hz payload;
            if (!decodePayload(payload_data, head.payload_size, &payload)) {
                return false;
            }
            applyPayload(payload, head, &snapshot);
            break;
        }
        case kPayloadType50Hz: {
            ClientUploadData50Hz payload;
            if (!decodePayload(payload_data, head.payload_size, &payload)) {
                return false;
            }
            applyPayload(payload, head, &snapshot);
            break;
        }
        case kPayloadType10Hz: {
            ClientUploadData10Hz payload;
            if (!decodePayload(payload_data, head.payload_size, &payload)) {
                return false;
            }
            applyPayload(payload, head, &snapshot);
            break;
        }
        case kPayloadType1Hz: {
            ClientUploadData1Hz payload;
            if (!decodePayload(payload_data, head.payload_size, &payload)) {
                return false;
            }
            applyPayload(payload, head, &snapshot);
            break;
        }
        default:
            return false;
    }

    storeLatest(snapshot);
    return true;
}

void RobotStateUdpReceiver::receiveLoop(bool high_rate) {
    std::array<unsigned char, 2048> buffer{};
    while (running_) {
        const ssize_t size = recv(socket_fd_, buffer.data(), buffer.size(), 0);
        if (size <= 0) {
            if (!running_) {
                break;
            }
            continue;
        }
        parsePacket(buffer.data(), static_cast<unsigned long>(size), high_rate);
    }
}

void RobotStateUdpReceiver::setListenPort(uint16_t listen_port) {
    listen_port_ = listen_port;
}

void RobotStateUdpReceiver::run() {
    receiveLoop(false);
}

bool RobotStateUdpReceiver::connect() {
    if (!openSocket()) {
        return false;
    }
    if (!running_.exchange(true)) {
        receive_thread_ = std::thread(&RobotStateUdpReceiver::run, this);
    }
    if (!has_snapshot_) {
        storeLatest(makeConnectedSnapshot());
    }
    return true;
}

bool RobotStateUdpReceiver::getLatestState(RobotStateSnapshot* snapshot) const {
    if (!snapshot) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_snapshot_) {
        return false;
    }
    *snapshot = latest_snapshot_;
    return true;
}

bool RobotStateUdpReceiver::getLatestState(RobotStateSnapshot* snapshot) {
    return static_cast<const RobotStateUdpReceiver*>(this)->getLatestState(snapshot);
}

void RobotStateUdpReceiver::storeLatest(const RobotStateSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_snapshot_ = snapshot;
    has_snapshot_ = true;
}

bool RobotStateUdpReceiver::getImuQuat(float* quat) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.imu_quat, quat);
}

bool RobotStateUdpReceiver::getLegOdom(LegOdom* leg_odom) const {
    RobotStateSnapshot snapshot;
    if (!leg_odom || !getLatestState(&snapshot)) {
        return false;
    }
    *leg_odom = snapshot.leg_odom;
    return true;
}

bool RobotStateUdpReceiver::getSubGait(unsigned char* sub_gait) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.sub_gait, sub_gait);
}

void RobotStateUdpReceiver::print200Hz(const ClientUploadPacketHead&, const ClientUploadData200Hz&) const {}

bool RobotStateUdpReceiver::getImuOmega(float* omega) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.imu_omega, omega);
}

bool RobotStateUdpReceiver::getLastGait(unsigned char* last_gait) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.last_gait, last_gait);
}

void RobotStateUdpReceiver::print1000Hz(const ClientUploadPacketHead&, const ClientUploadData1000Hz&) const {}

bool RobotStateUdpReceiver::getCurrentGait(unsigned char* current_gait) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.current_gait, current_gait);
}

std::array<float, 3> RobotStateUdpReceiver::getImuAccArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.imu_acc : std::array<float, 3>{};
}

std::array<float, 3> RobotStateUdpReceiver::getImuRpyArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.imu_rpy : std::array<float, 3>{};
}

bool RobotStateUdpReceiver::getJointTorque(float* joint_tau) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.joint_torque, joint_tau);
}

bool RobotStateUdpReceiver::getMaxVelocity(float* max_velocity) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.max_velocity, max_velocity);
}

bool RobotStateUdpReceiver::getBatteryLevel(unsigned char* battery_level) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.battery_level, battery_level);
}

std::array<float, 4> RobotStateUdpReceiver::getImuQuatArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.imu_quat : std::array<float, 4>{};
}

LegOdom RobotStateUdpReceiver::getLegOdomValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.leg_odom : LegOdom{};
}

unsigned char RobotStateUdpReceiver::getSubGaitValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.sub_gait : 0;
}

bool RobotStateUdpReceiver::getTimestamp1Hz(unsigned int* timestamp) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.battery_timestamp, timestamp);
}

std::array<float, 3> RobotStateUdpReceiver::getImuOmegaArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.imu_omega : std::array<float, 3>{};
}

bool RobotStateUdpReceiver::getJointPosition(float* joint_pos) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.joint_position, joint_pos);
}

bool RobotStateUdpReceiver::getJointVelocity(float* joint_vel) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.joint_velocity, joint_vel);
}

unsigned char RobotStateUdpReceiver::getLastGaitValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.last_gait : 0;
}

bool RobotStateUdpReceiver::getTimestamp10Hz(unsigned int* timestamp) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.motion_state_timestamp, timestamp);
}

bool RobotStateUdpReceiver::getTimestamp50Hz(unsigned int* timestamp) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.odometry_timestamp, timestamp);
}

bool RobotStateUdpReceiver::getBatteryCurrent(float* battery_current) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.battery_current, battery_current);
}

bool RobotStateUdpReceiver::getTimestamp200Hz(unsigned int* timestamp) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.imu_timestamp, timestamp);
}

bool RobotStateUdpReceiver::getLastMotionState(unsigned char* last_motion_state) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.last_motion_state, last_motion_state);
}

bool RobotStateUdpReceiver::getTimestamp1000Hz(unsigned int* timestamp) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.joint_state_timestamp, timestamp);
}

unsigned char RobotStateUdpReceiver::getCurrentGaitValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.current_gait : 0;
}

std::array<float, 12> RobotStateUdpReceiver::getJointTorqueArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.joint_torque : std::array<float, 12>{};
}

std::array<float, 3> RobotStateUdpReceiver::getMaxVelocityArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.max_velocity : std::array<float, 3>{};
}

bool RobotStateUdpReceiver::getMotorTemperature(float* motor_temperature) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.motor_temperature, motor_temperature);
}

unsigned char RobotStateUdpReceiver::getBatteryLevelValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.battery_level : 0;
}

bool RobotStateUdpReceiver::getDriverTemperature(float* driver_temperature) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.driver_temperature, driver_temperature);
}

unsigned int RobotStateUdpReceiver::getTimestamp1HzValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.battery_timestamp : 0;
}

bool RobotStateUdpReceiver::getCurrentMotionState(unsigned char* current_motion_state) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyScalar(snapshot.current_motion_state, current_motion_state);
}

std::array<float, 12> RobotStateUdpReceiver::getJointPositionArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.joint_position : std::array<float, 12>{};
}

std::array<float, 12> RobotStateUdpReceiver::getJointVelocityArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.joint_velocity : std::array<float, 12>{};
}

unsigned int RobotStateUdpReceiver::getTimestamp10HzValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.motion_state_timestamp : 0;
}

unsigned int RobotStateUdpReceiver::getTimestamp50HzValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.odometry_timestamp : 0;
}

float RobotStateUdpReceiver::getBatteryCurrentValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.battery_current : 0.0f;
}

unsigned int RobotStateUdpReceiver::getTimestamp200HzValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.imu_timestamp : 0;
}

unsigned char RobotStateUdpReceiver::getLastMotionStateValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.last_motion_state : 0;
}

unsigned int RobotStateUdpReceiver::getTimestamp1000HzValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.joint_state_timestamp : 0;
}

std::array<float, 12> RobotStateUdpReceiver::getMotorTemperatureArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.motor_temperature : std::array<float, 12>{};
}

std::array<float, 12> RobotStateUdpReceiver::getDriverTemperatureArray() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.driver_temperature : std::array<float, 12>{};
}

unsigned char RobotStateUdpReceiver::getCurrentMotionStateValue() const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) ? snapshot.current_motion_state : 0;
}

void RobotStateUdpReceiver::print1Hz(const ClientUploadPacketHead&, const ClientUploadData1Hz&) const {}

bool RobotStateUdpReceiver::getImuAcc(float* acc) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.imu_acc, acc);
}

bool RobotStateUdpReceiver::getImuRpy(float* rpy) const {
    RobotStateSnapshot snapshot;
    return getLatestState(&snapshot) && copyArray(snapshot.imu_rpy, rpy);
}

void RobotStateUdpReceiver::print10Hz(const ClientUploadPacketHead&, const ClientUploadData10Hz&) const {}

void RobotStateUdpReceiver::print50Hz(const ClientUploadPacketHead&, const ClientUploadData50Hz&) const {}

}  // namespace bpx_sdk

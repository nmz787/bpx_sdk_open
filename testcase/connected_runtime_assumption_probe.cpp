#include "../include/joint_level_control.h"
#include "../include/request_robot_state.h"
#include "../src/recovery_runtime.h"

#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr uint16_t kTcpServerPort = 10860;
constexpr uint16_t kPayloadType1000Hz = 0x1000;
constexpr uint16_t kPayloadType200Hz = 0x0200;
constexpr uint16_t kPayloadType50Hz = 0x0050;
constexpr uint16_t kPayloadType10Hz = 0x0010;
constexpr uint16_t kPayloadType1Hz = 0x0001;

bool closeEnough(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1e-5f;
}

int fail(const char* message) {
    std::cerr << message << '\n';
    return 1;
}

template <typename T>
void printValue(const char* key, T value) {
    std::cout << key << '=' << value << '\n';
}

bool bindLoopbackSocket(int fd, uint16_t port) {
    const int enabled = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    sockaddr_in address{};
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    return bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0 &&
           listen(fd, 1) == 0;
}

uint16_t reserveLoopbackUdpPort() {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return 0;
    }
    sockaddr_in address{};
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    if (bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        close(fd);
        return 0;
    }
    socklen_t size = sizeof(address);
    if (getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) != 0) {
        close(fd);
        return 0;
    }
    const uint16_t port = ntohs(address.sin_port);
    close(fd);
    return port;
}

template <typename T>
bool recvExact(int fd, T* value) {
    unsigned char* buffer = reinterpret_cast<unsigned char*>(value);
    size_t received = 0;
    while (received < sizeof(T)) {
        const ssize_t chunk = recv(fd, buffer + received, sizeof(T) - received, 0);
        if (chunk <= 0) {
            return false;
        }
        received += static_cast<size_t>(chunk);
    }
    return true;
}

template <typename Payload>
bool sendUploadPacket(int fd, uint16_t port, uint32_t timestamp_ms, uint16_t payload_type,
                      const Payload& payload) {
    struct Packet {
        bpx_sdk::ClientUploadPacketHead head{};
        Payload payload{};
    } packet{};
    packet.head.payload_type = payload_type;
    packet.head.payload_size = sizeof(Payload);
    packet.head.timestamp_ms = timestamp_ms;
    packet.payload = payload;

    sockaddr_in address{};
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return sendto(fd, &packet, sizeof(packet), 0,
                  reinterpret_cast<const sockaddr*>(&address),
                  sizeof(address)) == static_cast<ssize_t>(sizeof(packet));
}

struct SubscribeCapture {
    uint16_t robot_state_upload_port = 0;
    uint16_t joint_state_upload_port = 0;
    uint16_t robot_state_upload_rate_hz = 0;
    uint8_t host_server_mode = 0;
};

struct StatePayloads {
    bpx_sdk::ClientUploadData1000Hz data1000{};
    bpx_sdk::ClientUploadData200Hz data200{};
    bpx_sdk::ClientUploadData50Hz data50{};
    bpx_sdk::ClientUploadData10Hz data10{};
    bpx_sdk::ClientUploadData1Hz data1{};
};

struct ObservedState {
    float joint_position0 = 0.0f;
    float joint_velocity1 = 0.0f;
    float joint_torque2 = 0.0f;
    float imu_rpy2 = 0.0f;
    float imu_quat3 = 0.0f;
    float imu_acc1 = 0.0f;
    float imu_omega0 = 0.0f;
    float leg_velocity0 = 0.0f;
    float leg_position1 = 0.0f;
    float leg_orientation3 = 0.0f;
    float leg_omega2 = 0.0f;
    float motor_temperature0 = 0.0f;
    float driver_temperature1 = 0.0f;
    float max_velocity0 = 0.0f;
    uint8_t battery_level = 0;
    float battery_current = 0.0f;
    uint8_t current_motion_state = 0;
    uint8_t current_gait = 0;
    uint8_t last_motion_state = 0;
    uint8_t last_gait = 0;
    uint8_t sub_gait = 0;
    uint32_t joint_timestamp = 0;
    uint32_t imu_timestamp = 0;
    uint32_t odometry_timestamp = 0;
    uint32_t motion_timestamp = 0;
    uint32_t battery_timestamp = 0;
};

struct ObservedHighRateState {
    float joint_position0 = 0.0f;
    float joint_velocity1 = 0.0f;
    float joint_torque2 = 0.0f;
    float imu_rpy2 = 0.0f;
    float imu_quat3 = 0.0f;
    float imu_acc1 = 0.0f;
    float imu_omega0 = 0.0f;
    float timestamp = 0.0f;
    uint32_t seq = 0;
};

StatePayloads makeStatePayloads() {
    StatePayloads payloads{};
    payloads.data1000.joint_position[0] = 1.25f;
    payloads.data1000.joint_velocity[1] = -2.5f;
    payloads.data1000.joint_torque[2] = 3.75f;

    payloads.data200.imu_rpy = {0.4f, -0.5f, 0.6f};
    payloads.data200.imu_quat = {0.1f, 0.2f, 0.3f, 0.9f};
    payloads.data200.imu_acc = {1.0f, 2.0f, 3.0f};
    payloads.data200.imu_omega = {-4.0f, -5.0f, -6.0f};

    payloads.data50.leg_odom.velocity_body[0] = 0.7f;
    payloads.data50.leg_odom.position[1] = 5.0f;
    payloads.data50.leg_odom.orientation[3] = 0.8f;
    payloads.data50.leg_odom.angular_velocity[2] = -0.9f;

    payloads.data10.current_motion_state = static_cast<uint8_t>(bpx_sdk::MotionState::Motion);
    payloads.data10.current_gait = static_cast<uint8_t>(bpx_sdk::MotionGait::Running);
    payloads.data10.last_motion_state = static_cast<uint8_t>(bpx_sdk::MotionState::Passive);
    payloads.data10.last_gait = static_cast<uint8_t>(bpx_sdk::MotionGait::Walk);
    payloads.data10.sub_gait = 4;
    payloads.data10.max_velocity = {3.0f, 1.0f, 2.0f};

    payloads.data1.battery_level = 73;
    payloads.data1.battery_current = 6.5f;
    payloads.data1.motor_temperature[0] = 11;
    payloads.data1.driver_temperature[1] = -2;
    return payloads;
}

bpx_sdk::JointStatePacket makeJointFeedback() {
    bpx_sdk::JointStatePacket feedback{};
    feedback.joint_position[0] = 3.25f;
    feedback.joint_velocity[1] = -2.5f;
    feedback.joint_torque[2] = 1.5f;
    feedback.imu_rpy = {0.4f, -0.5f, 0.6f};
    feedback.imu_quat = {0.1f, 0.2f, 0.3f, 0.9f};
    feedback.imu_acc = {1.0f, 2.0f, 3.0f};
    feedback.imu_omega = {-4.0f, -5.0f, -6.0f};
    feedback.timestamp_ms = 4321.0f;
    feedback.seq = 77;
    return feedback;
}

std::pair<int, std::thread> runSubscribeServer(SubscribeCapture* request) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0 || !bindLoopbackSocket(server_fd, kTcpServerPort)) {
        return {-1, std::thread()};
    }
    return {
        server_fd,
        std::thread([server_fd, request] {
            sockaddr_in peer{};
            socklen_t peer_size = sizeof(peer);
            const int client_fd =
                accept(server_fd, reinterpret_cast<sockaddr*>(&peer), &peer_size);
            if (client_fd < 0) {
                return;
            }
            bpx_sdk::SubscribeStateReq wire_request{};
            if (recvExact(client_fd, &wire_request) && request) {
                request->robot_state_upload_port = wire_request.robot_state_upload_port;
                request->joint_state_upload_port = wire_request.joint_state_upload_port;
                request->robot_state_upload_rate_hz = wire_request.robot_state_upload_rate_hz;
                request->host_server_mode = wire_request.host_server_mode;
                bpx_sdk::SubscribeStateResp response{};
                response.raw[0] = 1;
                send(client_fd, response.raw.data(), response.raw.size(), 0);
            }
            close(client_fd);
        }),
    };
}

void closeServer(std::pair<int, std::thread>* server) {
    if (!server) {
        return;
    }
    if (server->first >= 0) {
        close(server->first);
        server->first = -1;
    }
    if (server->second.joinable()) {
        server->second.join();
    }
}

bool waitForRobotState(const bpx_sdk::RequestRobotState& state, const StatePayloads& payloads,
                       ObservedState* observed) {
    if (!observed) {
        return false;
    }
    for (int attempt = 0; attempt < 40; ++attempt) {
        float joint_pos[12] = {};
        float joint_vel[12] = {};
        float joint_tau[12] = {};
        float imu_rpy[3] = {};
        float imu_quat[4] = {};
        float imu_acc[3] = {};
        float imu_omega[3] = {};
        bpx_sdk::LegOdom leg_odom{};
        float motor_temperature[12] = {};
        float driver_temperature[12] = {};
        float max_velocity[3] = {};
        uint8_t battery_level = 0;
        float battery_current = 0.0f;
        uint8_t current_motion_state = 0;
        uint8_t current_gait = 0;
        uint8_t last_motion_state = 0;
        uint8_t last_gait = 0;
        uint8_t sub_gait = 0;
        uint32_t joint_timestamp = 0;
        uint32_t imu_timestamp = 0;
        uint32_t odometry_timestamp = 0;
        uint32_t motion_timestamp = 0;
        uint32_t battery_timestamp = 0;
        if (state.getJointPosition(joint_pos) &&
            state.getJointVelocity(joint_vel) &&
            state.getJointTorque(joint_tau) &&
            state.getImuRpy(imu_rpy) &&
            state.getImuQuat(imu_quat) &&
            state.getImuAcc(imu_acc) &&
            state.getImuOmega(imu_omega) &&
            state.getLegOdom(&leg_odom) &&
            state.getMotorTemperature(motor_temperature) &&
            state.getDriverTemperature(driver_temperature) &&
            state.getMaxVelocity(max_velocity) &&
            state.getBatteryLevel(&battery_level) &&
            state.getBatteryCurrent(&battery_current) &&
            state.getCurrentMotionState(&current_motion_state) &&
            state.getCurrentGait(&current_gait) &&
            state.getLastMotionState(&last_motion_state) &&
            state.getLastGait(&last_gait) &&
            state.getSubGait(&sub_gait) &&
            state.getJointStateTimestamp(&joint_timestamp) &&
            state.getImuTimestamp(&imu_timestamp) &&
            state.getOdometryTimestamp(&odometry_timestamp) &&
            state.getMotionStateTimestamp(&motion_timestamp) &&
            state.getBatteryTimestamp(&battery_timestamp) &&
            closeEnough(joint_pos[0], payloads.data1000.joint_position[0]) &&
            closeEnough(joint_vel[1], payloads.data1000.joint_velocity[1]) &&
            closeEnough(joint_tau[2], payloads.data1000.joint_torque[2]) &&
            closeEnough(imu_rpy[2], payloads.data200.imu_rpy[2]) &&
            closeEnough(imu_quat[3], payloads.data200.imu_quat[3]) &&
            closeEnough(imu_acc[1], payloads.data200.imu_acc[1]) &&
            closeEnough(imu_omega[0], payloads.data200.imu_omega[0]) &&
            closeEnough(leg_odom.velocity_body[0], payloads.data50.leg_odom.velocity_body[0]) &&
            closeEnough(leg_odom.position[1], payloads.data50.leg_odom.position[1]) &&
            closeEnough(leg_odom.orientation[3], payloads.data50.leg_odom.orientation[3]) &&
            closeEnough(leg_odom.angular_velocity[2],
                        payloads.data50.leg_odom.angular_velocity[2]) &&
            closeEnough(motor_temperature[0], 71.0f) &&
            closeEnough(driver_temperature[1], 58.0f) &&
            closeEnough(max_velocity[0], payloads.data10.max_velocity[0]) &&
            battery_level == payloads.data1.battery_level &&
            closeEnough(battery_current, payloads.data1.battery_current) &&
            current_motion_state == payloads.data10.current_motion_state &&
            current_gait == payloads.data10.current_gait &&
            last_motion_state == payloads.data10.last_motion_state &&
            last_gait == payloads.data10.last_gait &&
            sub_gait == static_cast<uint8_t>(payloads.data10.sub_gait) &&
            joint_timestamp == 1001 &&
            imu_timestamp == 1002 &&
            odometry_timestamp == 1003 &&
            motion_timestamp == 1004 &&
            battery_timestamp == 1005) {
            observed->joint_position0 = joint_pos[0];
            observed->joint_velocity1 = joint_vel[1];
            observed->joint_torque2 = joint_tau[2];
            observed->imu_rpy2 = imu_rpy[2];
            observed->imu_quat3 = imu_quat[3];
            observed->imu_acc1 = imu_acc[1];
            observed->imu_omega0 = imu_omega[0];
            observed->leg_velocity0 = leg_odom.velocity_body[0];
            observed->leg_position1 = leg_odom.position[1];
            observed->leg_orientation3 = leg_odom.orientation[3];
            observed->leg_omega2 = leg_odom.angular_velocity[2];
            observed->motor_temperature0 = motor_temperature[0];
            observed->driver_temperature1 = driver_temperature[1];
            observed->max_velocity0 = max_velocity[0];
            observed->battery_level = battery_level;
            observed->battery_current = battery_current;
            observed->current_motion_state = current_motion_state;
            observed->current_gait = current_gait;
            observed->last_motion_state = last_motion_state;
            observed->last_gait = last_gait;
            observed->sub_gait = sub_gait;
            observed->joint_timestamp = joint_timestamp;
            observed->imu_timestamp = imu_timestamp;
            observed->odometry_timestamp = odometry_timestamp;
            observed->motion_timestamp = motion_timestamp;
            observed->battery_timestamp = battery_timestamp;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return false;
}

bool waitForJointHighRateState(const bpx_sdk::JointLevelControl& joint,
                               const bpx_sdk::JointStatePacket& feedback,
                               ObservedHighRateState* observed) {
    if (!observed) {
        return false;
    }
    for (int attempt = 0; attempt < 40; ++attempt) {
        float joint_pos[12] = {};
        float joint_vel[12] = {};
        float joint_tau[12] = {};
        float imu_rpy[3] = {};
        float imu_quat[4] = {};
        float imu_acc[3] = {};
        float imu_omega[3] = {};
        float timestamp = 0.0f;
        uint32_t seq = 0;
        if (joint.getJointPositionHighRate(joint_pos) &&
            joint.getJointVelocityHighRate(joint_vel) &&
            joint.getJointTorqueHighRate(joint_tau) &&
            joint.getImuRpyHighRate(imu_rpy) &&
            joint.getImuQuatHighRate(imu_quat) &&
            joint.getImuAccHighRate(imu_acc) &&
            joint.getImuOmegaHighRate(imu_omega) &&
            joint.getJointStateTimestampHighRate(&timestamp) &&
            joint.getJointStateSeqHighRate(&seq) &&
            closeEnough(joint_pos[0], feedback.joint_position[0]) &&
            closeEnough(joint_vel[1], feedback.joint_velocity[1]) &&
            closeEnough(joint_tau[2], feedback.joint_torque[2]) &&
            closeEnough(imu_rpy[2], feedback.imu_rpy[2]) &&
            closeEnough(imu_quat[3], feedback.imu_quat[3]) &&
            closeEnough(imu_acc[1], feedback.imu_acc[1]) &&
            closeEnough(imu_omega[0], feedback.imu_omega[0]) &&
            closeEnough(timestamp, feedback.timestamp_ms) &&
            seq == feedback.seq) {
            observed->joint_position0 = joint_pos[0];
            observed->joint_velocity1 = joint_vel[1];
            observed->joint_torque2 = joint_tau[2];
            observed->imu_rpy2 = imu_rpy[2];
            observed->imu_quat3 = imu_quat[3];
            observed->imu_acc1 = imu_acc[1];
            observed->imu_omega0 = imu_omega[0];
            observed->timestamp = timestamp;
            observed->seq = seq;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return false;
}

void printObservedState(const char* prefix, const ObservedState& observed) {
    printValue((std::string(prefix) + ".joint_position0").c_str(), observed.joint_position0);
    printValue((std::string(prefix) + ".joint_velocity1").c_str(), observed.joint_velocity1);
    printValue((std::string(prefix) + ".joint_torque2").c_str(), observed.joint_torque2);
    printValue((std::string(prefix) + ".imu_rpy2").c_str(), observed.imu_rpy2);
    printValue((std::string(prefix) + ".imu_quat3").c_str(), observed.imu_quat3);
    printValue((std::string(prefix) + ".imu_acc1").c_str(), observed.imu_acc1);
    printValue((std::string(prefix) + ".imu_omega0").c_str(), observed.imu_omega0);
    printValue((std::string(prefix) + ".leg_velocity0").c_str(), observed.leg_velocity0);
    printValue((std::string(prefix) + ".leg_position1").c_str(), observed.leg_position1);
    printValue((std::string(prefix) + ".leg_orientation3").c_str(),
               observed.leg_orientation3);
    printValue((std::string(prefix) + ".leg_omega2").c_str(), observed.leg_omega2);
    printValue((std::string(prefix) + ".motor_temperature0").c_str(),
               observed.motor_temperature0);
    printValue((std::string(prefix) + ".driver_temperature1").c_str(),
               observed.driver_temperature1);
    printValue((std::string(prefix) + ".max_velocity0").c_str(), observed.max_velocity0);
    printValue((std::string(prefix) + ".battery_level").c_str(),
               static_cast<int>(observed.battery_level));
    printValue((std::string(prefix) + ".battery_current").c_str(), observed.battery_current);
    printValue((std::string(prefix) + ".current_motion_state").c_str(),
               static_cast<int>(observed.current_motion_state));
    printValue((std::string(prefix) + ".current_gait").c_str(),
               static_cast<int>(observed.current_gait));
    printValue((std::string(prefix) + ".last_motion_state").c_str(),
               static_cast<int>(observed.last_motion_state));
    printValue((std::string(prefix) + ".last_gait").c_str(),
               static_cast<int>(observed.last_gait));
    printValue((std::string(prefix) + ".sub_gait").c_str(),
               static_cast<int>(observed.sub_gait));
    printValue((std::string(prefix) + ".joint_timestamp").c_str(), observed.joint_timestamp);
    printValue((std::string(prefix) + ".imu_timestamp").c_str(), observed.imu_timestamp);
    printValue((std::string(prefix) + ".odometry_timestamp").c_str(),
               observed.odometry_timestamp);
    printValue((std::string(prefix) + ".motion_timestamp").c_str(), observed.motion_timestamp);
    printValue((std::string(prefix) + ".battery_timestamp").c_str(),
               observed.battery_timestamp);
}

void printObservedHighRateState(const char* prefix, const ObservedHighRateState& observed) {
    printValue((std::string(prefix) + ".joint_position0").c_str(), observed.joint_position0);
    printValue((std::string(prefix) + ".joint_velocity1").c_str(), observed.joint_velocity1);
    printValue((std::string(prefix) + ".joint_torque2").c_str(), observed.joint_torque2);
    printValue((std::string(prefix) + ".imu_rpy2").c_str(), observed.imu_rpy2);
    printValue((std::string(prefix) + ".imu_quat3").c_str(), observed.imu_quat3);
    printValue((std::string(prefix) + ".imu_acc1").c_str(), observed.imu_acc1);
    printValue((std::string(prefix) + ".imu_omega0").c_str(), observed.imu_omega0);
    printValue((std::string(prefix) + ".timestamp").c_str(), observed.timestamp);
    printValue((std::string(prefix) + ".seq").c_str(), observed.seq);
}

}  // namespace

int main() {
    const StatePayloads state_payloads = makeStatePayloads();

    SubscribeCapture request_state{};
    const uint16_t robot_state_port = reserveLoopbackUdpPort();
    if (robot_state_port == 0) {
        return fail("failed to reserve RequestRobotState UDP port");
    }
    auto state_server = runSubscribeServer(&request_state);
    if (state_server.first < 0) {
        return fail("failed to start RequestRobotState subscribe server");
    }

    bpx_sdk::RequestRobotState state;
    state.setRobotIp("127.0.0.1");
    state.setRobotStateUploadPort(robot_state_port);
    if (!state.connect()) {
        closeServer(&state_server);
        return fail("RequestRobotState connect failed");
    }
    closeServer(&state_server);

    int state_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (state_udp_fd < 0) {
        state.disconnect();
        return fail("failed to open RequestRobotState UDP sender");
    }
    if (!sendUploadPacket(state_udp_fd, robot_state_port, 1001, kPayloadType1000Hz,
                          state_payloads.data1000) ||
        !sendUploadPacket(state_udp_fd, robot_state_port, 1002, kPayloadType200Hz,
                          state_payloads.data200) ||
        !sendUploadPacket(state_udp_fd, robot_state_port, 1003, kPayloadType50Hz,
                          state_payloads.data50) ||
        !sendUploadPacket(state_udp_fd, robot_state_port, 1004, kPayloadType10Hz,
                          state_payloads.data10) ||
        !sendUploadPacket(state_udp_fd, robot_state_port, 1005, kPayloadType1Hz,
                          state_payloads.data1)) {
        close(state_udp_fd);
        state.disconnect();
        return fail("failed to send RequestRobotState upload packets");
    }
    close(state_udp_fd);

    ObservedState observed_state{};
    if (!waitForRobotState(state, state_payloads, &observed_state)) {
        state.disconnect();
        return fail("RequestRobotState did not surface loopback robot-state traffic");
    }
    state.disconnect();

    SubscribeCapture request_joint{};
    const uint16_t joint_robot_state_port = reserveLoopbackUdpPort();
    const uint16_t joint_state_port = reserveLoopbackUdpPort();
    if (joint_robot_state_port == 0 || joint_state_port == 0 ||
        joint_robot_state_port == joint_state_port) {
        return fail("failed to reserve JointLevelControl UDP ports");
    }
    auto joint_server = runSubscribeServer(&request_joint);
    if (joint_server.first < 0) {
        return fail("failed to start JointLevelControl subscribe server");
    }

    bpx_sdk::JointLevelControl joint;
    joint.setRobotIp("127.0.0.1");
    joint.setRobotStateUploadPort(joint_robot_state_port);
    joint.setJointStateUploadPort(joint_state_port);
    if (!joint.connect()) {
        closeServer(&joint_server);
        return fail("JointLevelControl connect failed");
    }
    closeServer(&joint_server);

    int joint_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (joint_udp_fd < 0) {
        joint.disconnect();
        return fail("failed to open JointLevelControl UDP sender");
    }
    if (!sendUploadPacket(joint_udp_fd, joint_robot_state_port, 1001, kPayloadType1000Hz,
                          state_payloads.data1000) ||
        !sendUploadPacket(joint_udp_fd, joint_robot_state_port, 1002, kPayloadType200Hz,
                          state_payloads.data200) ||
        !sendUploadPacket(joint_udp_fd, joint_robot_state_port, 1003, kPayloadType50Hz,
                          state_payloads.data50) ||
        !sendUploadPacket(joint_udp_fd, joint_robot_state_port, 1004, kPayloadType10Hz,
                          state_payloads.data10) ||
        !sendUploadPacket(joint_udp_fd, joint_robot_state_port, 1005, kPayloadType1Hz,
                          state_payloads.data1)) {
        close(joint_udp_fd);
        joint.disconnect();
        return fail("failed to send JointLevelControl robot-state packets");
    }

    const bpx_sdk::JointStatePacket feedback = makeJointFeedback();
    sockaddr_in joint_address{};
    std::memset(&joint_address, 0, sizeof(joint_address));
    joint_address.sin_family = AF_INET;
    joint_address.sin_port = htons(joint_state_port);
    joint_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (sendto(joint_udp_fd, &feedback, sizeof(feedback), 0,
               reinterpret_cast<const sockaddr*>(&joint_address),
               sizeof(joint_address)) != static_cast<ssize_t>(sizeof(feedback))) {
        close(joint_udp_fd);
        joint.disconnect();
        return fail("failed to send JointLevelControl joint feedback");
    }
    close(joint_udp_fd);

    ObservedState observed_joint_state{};
    if (!waitForRobotState(joint, state_payloads, &observed_joint_state)) {
        joint.disconnect();
        return fail("JointLevelControl did not surface loopback robot-state traffic");
    }

    ObservedHighRateState observed_high_rate{};
    if (!waitForJointHighRateState(joint, feedback, &observed_high_rate)) {
        joint.disconnect();
        return fail("JointLevelControl did not surface loopback joint feedback");
    }
    joint.disconnect();

    printValue("request0.robot_state_port_matches",
               static_cast<int>(request_state.robot_state_upload_port == robot_state_port));
    printValue("request0.joint_state_port_matches",
               static_cast<int>(request_state.joint_state_upload_port ==
                                bpx_sdk::DEFAULT_CLIENT_JOINT_STATE_UDP_PORT));
    printValue("request0.robot_state_upload_rate_hz", request_state.robot_state_upload_rate_hz);
    printValue("request0.host_server_mode", static_cast<int>(request_state.host_server_mode));
    printObservedState("request0.state", observed_state);

    printValue("request1.robot_state_port_matches",
               static_cast<int>(request_joint.robot_state_upload_port ==
                                joint_robot_state_port));
    printValue("request1.joint_state_port_matches",
               static_cast<int>(request_joint.joint_state_upload_port == joint_state_port));
    printValue("request1.robot_state_upload_rate_hz", request_joint.robot_state_upload_rate_hz);
    printValue("request1.host_server_mode", static_cast<int>(request_joint.host_server_mode));
    printObservedState("request1.state", observed_joint_state);
    printObservedHighRateState("request1.high_rate", observed_high_rate);
    return 0;
}

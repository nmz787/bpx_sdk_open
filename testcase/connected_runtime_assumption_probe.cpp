#include "../src/recovery_runtime.h"
#include "../include/joint_level_control.h"
#include "../include/request_robot_state.h"

#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <optional>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint16_t kTcpServerPort = 10860;

bool closeEnough(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1e-5f;
}

template <typename T>
void printValue(const char* key, T value) {
    std::cout << key << '=' << value << '\n';
}

void printFloat(const char* key, float value) {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(6);
    stream << value;
    printValue(key, stream.str());
}

bool bindLoopbackSocket(int fd, uint16_t port, int type) {
    const int enabled = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    sockaddr_in address{};
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons(port);
    if (bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        return false;
    }
    if (type == SOCK_STREAM) {
        return listen(fd, 4) == 0;
    }
    return true;
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
bool sendUploadPacket(int fd, uint16_t port, uint32_t seq, uint32_t timestamp_ms,
                      uint16_t payload_type, const Payload& payload) {
    struct Packet {
        bpx_sdk::ClientUploadPacketHead head{};
        Payload payload{};
    } packet{};
    packet.head.seq = seq;
    packet.head.timestamp_ms = timestamp_ms;
    packet.head.payload_size = sizeof(Payload);
    packet.head.payload_type = payload_type;
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

template <typename Payload>
bool sendRawUdpPacket(int fd, uint16_t port, const Payload& payload) {
    sockaddr_in address{};
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return sendto(fd, &payload, sizeof(payload), 0,
                  reinterpret_cast<const sockaddr*>(&address),
                  sizeof(address)) == static_cast<ssize_t>(sizeof(payload));
}

struct SubscribeCapture {
    uint16_t session_id = 0;
    uint16_t robot_state_upload_port = 0;
    uint16_t joint_state_upload_port = 0;
    uint16_t robot_state_upload_rate_hz = 0;
    uint8_t host_server_mode = 0;
};

}  // namespace

int main() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0 || !bindLoopbackSocket(server_fd, kTcpServerPort, SOCK_STREAM)) {
        return 1;
    }

    const uint16_t robot_state_port = reserveLoopbackUdpPort();
    const uint16_t joint_state_port = reserveLoopbackUdpPort();
    if (robot_state_port == 0 || joint_state_port == 0 || robot_state_port == joint_state_port) {
        close(server_fd);
        return 1;
    }

    std::vector<SubscribeCapture> requests;
    bool server_running = true;
    std::thread server([&] {
        while (server_running && requests.size() < 2) {
            sockaddr_in peer{};
            socklen_t peer_size = sizeof(peer);
            const int client_fd =
                accept(server_fd, reinterpret_cast<sockaddr*>(&peer), &peer_size);
            if (client_fd < 0) {
                continue;
            }
            bpx_sdk::SubscribeStateReq request{};
            if (recvExact(client_fd, &request)) {
                requests.push_back(SubscribeCapture{
                    request.session_id,
                    request.robot_state_upload_port,
                    request.joint_state_upload_port,
                    request.robot_state_upload_rate_hz,
                    request.host_server_mode,
                });
                bpx_sdk::SubscribeStateResp response{};
                response.raw[0] = 1;
                send(client_fd, response.raw.data(), response.raw.size(), 0);
            }
            close(client_fd);
        }
    });

    bpx_sdk::RequestRobotState state;
    state.setRobotIp("127.0.0.1");
    state.setRobotStateUploadPort(robot_state_port);
    if (!state.connect()) {
        server_running = false;
        close(server_fd);
        server.join();
        return 1;
    }

    bpx_sdk::JointLevelControl joint;
    joint.setRobotIp("127.0.0.1");
    joint.setRobotStateUploadPort(robot_state_port);
    joint.setJointStateUploadPort(joint_state_port);
    if (!joint.connect()) {
        server_running = false;
        close(server_fd);
        server.join();
        return 1;
    }

    for (int attempt = 0; attempt < 40 && requests.size() < 2; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    server_running = false;
    shutdown(server_fd, SHUT_RDWR);
    close(server_fd);
    server.join();
    if (requests.size() < 2) {
        return 1;
    }

    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        return 1;
    }

    bpx_sdk::ClientUploadData1000Hz data1000{};
    data1000.joint_position[0] = 1.25f;
    data1000.joint_velocity[1] = -2.5f;
    data1000.joint_torque[2] = 3.75f;
    bpx_sdk::ClientUploadData200Hz data200{};
    data200.imu_rpy = {0.4f, -0.5f, 0.6f};
    data200.imu_quat = {0.1f, 0.2f, 0.3f, 0.9f};
    data200.imu_acc = {1.0f, 2.0f, 3.0f};
    data200.imu_omega = {-4.0f, -5.0f, -6.0f};
    bpx_sdk::ClientUploadData50Hz data50{};
    data50.leg_odom.velocity_body[0] = 0.7f;
    data50.leg_odom.position[1] = 5.0f;
    data50.leg_odom.orientation[3] = 0.8f;
    data50.leg_odom.angular_velocity[2] = -0.9f;
    bpx_sdk::ClientUploadData10Hz data10{};
    data10.current_motion_state = static_cast<uint8_t>(bpx_sdk::MotionState::Motion);
    data10.current_gait = static_cast<uint8_t>(bpx_sdk::MotionGait::Running);
    data10.last_motion_state = static_cast<uint8_t>(bpx_sdk::MotionState::Passive);
    data10.last_gait = static_cast<uint8_t>(bpx_sdk::MotionGait::Walk);
    data10.sub_gait = 4;
    data10.max_velocity = {3.0f, 1.0f, 2.0f};
    bpx_sdk::ClientUploadData1Hz data1{};
    data1.battery_level = 73;
    data1.battery_current = 6.5f;
    data1.motor_temperature[0] = 11;
    data1.driver_temperature[1] = -2;
    bpx_sdk::JointStatePacket joint_packet{};
    joint_packet.joint_position[0] = 3.25f;
    joint_packet.joint_velocity[1] = -2.5f;
    joint_packet.joint_torque[2] = 1.5f;
    joint_packet.imu_rpy = {0.4f, -0.5f, 0.6f};
    joint_packet.imu_quat = {0.1f, 0.2f, 0.3f, 0.9f};
    joint_packet.imu_acc = {1.0f, 2.0f, 3.0f};
    joint_packet.imu_omega = {-4.0f, -5.0f, -6.0f};
    joint_packet.timestamp_ms = 4321.0f;
    joint_packet.seq = 77;

    if (!sendUploadPacket(udp_fd, robot_state_port, 11, 1001, 0x1000, data1000) ||
        !sendUploadPacket(udp_fd, robot_state_port, 12, 1002, 0x0200, data200) ||
        !sendUploadPacket(udp_fd, robot_state_port, 13, 1003, 0x0050, data50) ||
        !sendUploadPacket(udp_fd, robot_state_port, 14, 1004, 0x0010, data10) ||
        !sendUploadPacket(udp_fd, robot_state_port, 15, 1005, 0x0001, data1) ||
        !sendRawUdpPacket(udp_fd, joint_state_port, joint_packet)) {
        close(udp_fd);
        return 1;
    }
    close(udp_fd);

    bool received = false;
    std::array<float, 12> joint_pos{};
    std::array<float, 12> joint_vel{};
    std::array<float, 12> joint_tau{};
    std::array<float, 3> imu_rpy{};
    std::array<float, 4> imu_quat{};
    std::array<float, 3> imu_acc{};
    std::array<float, 3> imu_omega{};
    bpx_sdk::LegOdom leg_odom{};
    std::array<float, 12> motor_temperature{};
    std::array<float, 12> driver_temperature{};
    std::array<float, 3> max_velocity{};
    uint8_t battery_level = 0;
    float battery_current = 0.0f;
    uint8_t current_motion_state = 0;
    uint8_t current_gait = 0;
    uint8_t last_motion_state = 0;
    uint8_t last_gait = 0;
    uint8_t sub_gait = 0;
    uint32_t joint_ts = 0;
    uint32_t imu_ts = 0;
    uint32_t odom_ts = 0;
    uint32_t motion_ts = 0;
    uint32_t battery_ts = 0;
    std::array<float, 12> high_rate_pos{};
    std::array<float, 12> high_rate_vel{};
    std::array<float, 12> high_rate_tau{};
    std::array<float, 3> high_rate_rpy{};
    std::array<float, 4> high_rate_quat{};
    std::array<float, 3> high_rate_acc{};
    std::array<float, 3> high_rate_omega{};
    float high_rate_timestamp = 0.0f;
    uint32_t high_rate_seq = 0;

    for (int attempt = 0; attempt < 40; ++attempt) {
        received =
            state.getJointPosition(joint_pos.data()) &&
            state.getJointVelocity(joint_vel.data()) &&
            state.getJointTorque(joint_tau.data()) &&
            state.getImuRpy(imu_rpy.data()) &&
            state.getImuQuat(imu_quat.data()) &&
            state.getImuAcc(imu_acc.data()) &&
            state.getImuOmega(imu_omega.data()) &&
            state.getLegOdom(&leg_odom) &&
            state.getMotorTemperature(motor_temperature.data()) &&
            state.getDriverTemperature(driver_temperature.data()) &&
            state.getMaxVelocity(max_velocity.data()) &&
            state.getBatteryLevel(&battery_level) &&
            state.getBatteryCurrent(&battery_current) &&
            state.getCurrentMotionState(&current_motion_state) &&
            state.getCurrentGait(&current_gait) &&
            state.getLastMotionState(&last_motion_state) &&
            state.getLastGait(&last_gait) &&
            state.getSubGait(&sub_gait) &&
            state.getJointStateTimestamp(&joint_ts) &&
            state.getImuTimestamp(&imu_ts) &&
            state.getOdometryTimestamp(&odom_ts) &&
            state.getMotionStateTimestamp(&motion_ts) &&
            state.getBatteryTimestamp(&battery_ts) &&
            joint.getJointPositionHighRate(high_rate_pos.data()) &&
            joint.getJointVelocityHighRate(high_rate_vel.data()) &&
            joint.getJointTorqueHighRate(high_rate_tau.data()) &&
            joint.getImuRpyHighRate(high_rate_rpy.data()) &&
            joint.getImuQuatHighRate(high_rate_quat.data()) &&
            joint.getImuAccHighRate(high_rate_acc.data()) &&
            joint.getImuOmegaHighRate(high_rate_omega.data()) &&
            joint.getJointStateTimestampHighRate(&high_rate_timestamp) &&
            joint.getJointStateSeqHighRate(&high_rate_seq);
        if (received &&
            closeEnough(joint_pos[0], data1000.joint_position[0]) &&
            closeEnough(joint_vel[1], data1000.joint_velocity[1]) &&
            closeEnough(joint_tau[2], data1000.joint_torque[2]) &&
            closeEnough(high_rate_pos[0], joint_packet.joint_position[0]) &&
            closeEnough(high_rate_vel[1], joint_packet.joint_velocity[1]) &&
            closeEnough(high_rate_tau[2], joint_packet.joint_torque[2])) {
            break;
        }
        received = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    if (!received) {
        return 1;
    }

    printValue("request0.robot_state_upload_port", requests[0].robot_state_upload_port);
    printValue("request0.joint_state_upload_port", requests[0].joint_state_upload_port);
    printValue("request0.robot_state_upload_rate_hz", requests[0].robot_state_upload_rate_hz);
    printValue("request0.host_server_mode", static_cast<int>(requests[0].host_server_mode));
    printValue("request1.robot_state_upload_port", requests[1].robot_state_upload_port);
    printValue("request1.joint_state_upload_port", requests[1].joint_state_upload_port);
    printValue("request1.robot_state_upload_rate_hz", requests[1].robot_state_upload_rate_hz);
    printValue("request1.host_server_mode", static_cast<int>(requests[1].host_server_mode));
    printFloat("state.joint_position0", joint_pos[0]);
    printFloat("state.joint_velocity1", joint_vel[1]);
    printFloat("state.joint_torque2", joint_tau[2]);
    printFloat("state.imu_rpy2", imu_rpy[2]);
    printFloat("state.imu_quat3", imu_quat[3]);
    printFloat("state.imu_acc1", imu_acc[1]);
    printFloat("state.imu_omega0", imu_omega[0]);
    printFloat("state.leg_velocity0", leg_odom.velocity_body[0]);
    printFloat("state.leg_position1", leg_odom.position[1]);
    printFloat("state.leg_orientation3", leg_odom.orientation[3]);
    printFloat("state.leg_omega2", leg_odom.angular_velocity[2]);
    printFloat("state.motor_temperature0", motor_temperature[0]);
    printFloat("state.driver_temperature1", driver_temperature[1]);
    printFloat("state.max_velocity0", max_velocity[0]);
    printValue("state.battery_level", static_cast<int>(battery_level));
    printFloat("state.battery_current", battery_current);
    printValue("state.current_motion_state", static_cast<int>(current_motion_state));
    printValue("state.current_gait", static_cast<int>(current_gait));
    printValue("state.last_motion_state", static_cast<int>(last_motion_state));
    printValue("state.last_gait", static_cast<int>(last_gait));
    printValue("state.sub_gait", static_cast<int>(sub_gait));
    printValue("state.joint_timestamp", joint_ts);
    printValue("state.imu_timestamp", imu_ts);
    printValue("state.odom_timestamp", odom_ts);
    printValue("state.motion_timestamp", motion_ts);
    printValue("state.battery_timestamp", battery_ts);
    printFloat("joint.position0", high_rate_pos[0]);
    printFloat("joint.velocity1", high_rate_vel[1]);
    printFloat("joint.torque2", high_rate_tau[2]);
    printFloat("joint.imu_rpy2", high_rate_rpy[2]);
    printFloat("joint.imu_quat3", high_rate_quat[3]);
    printFloat("joint.imu_acc1", high_rate_acc[1]);
    printFloat("joint.imu_omega0", high_rate_omega[0]);
    printFloat("joint.timestamp", high_rate_timestamp);
    printValue("joint.seq", high_rate_seq);
    return 0;
}

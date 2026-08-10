#include "../src/joint_command_sender.h"
#include "../src/motion_command_sender.h"
#include "../src/tcp_subscribe_client.h"
#include "../include/joint_level_control.h"
#include "../include/request_robot_state.h"

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <optional>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr uint16_t kTcpServerPort = 10860;
constexpr uint16_t kTcpLocalPort = 16060;
constexpr uint16_t kMotionCommandPort = 9527;
constexpr uint16_t kJointCommandPort = 7896;

struct MotionCommandWirePacket {
    uint32_t seq = 0;
    uint8_t command = 0;
    uint8_t gait = 0;
    uint8_t reserved0 = 0;
    uint8_t reserved1 = 0;
    std::array<float, 6> values{};
    uint8_t velocity_control_enabled = 0;
    uint8_t zero_positions_nonce = 0;
    uint8_t sub_gait = 0;
    uint8_t reserved2 = 0;
    uint32_t control_flags = 0;
    std::array<uint8_t, 16> reserved_tail{};
};

static_assert(sizeof(MotionCommandWirePacket) == 56, "unexpected motion packet size");

bool closeEnough(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1e-5f;
}

int fail(const char* message) {
    std::cerr << message << '\n';
    return 1;
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
        return listen(fd, 1) == 0;
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

template <typename T>
bool recvUdpExact(int fd, T* value) {
    sockaddr_in peer{};
    socklen_t peer_size = sizeof(peer);
    const ssize_t size =
        recvfrom(fd, value, sizeof(T), 0, reinterpret_cast<sockaddr*>(&peer), &peer_size);
    return size == static_cast<ssize_t>(sizeof(T));
}

void encodeWord(std::array<uint8_t, 32>* raw, size_t index, uint32_t value) {
    if (!raw || index >= 7) {
        return;
    }
    const size_t base = 4 + index * 4;
    (*raw)[base] = static_cast<uint8_t>(value & 0xffu);
    (*raw)[base + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
    (*raw)[base + 2] = static_cast<uint8_t>((value >> 16) & 0xffu);
    (*raw)[base + 3] = static_cast<uint8_t>((value >> 24) & 0xffu);
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

}  // namespace

int main() {
    {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0 || !bindLoopbackSocket(server_fd, kTcpServerPort, SOCK_STREAM)) {
            return fail("failed to open loopback TCP server");
        }

        std::optional<bpx_sdk::SubscribeStateReq> captured_request;
        std::optional<uint16_t> observed_peer_port;
        std::thread server([&] {
            sockaddr_in peer{};
            socklen_t peer_size = sizeof(peer);
            const int client_fd =
                accept(server_fd, reinterpret_cast<sockaddr*>(&peer), &peer_size);
            if (client_fd < 0) {
                return;
            }
            observed_peer_port = ntohs(peer.sin_port);
            bpx_sdk::SubscribeStateReq request{};
            if (recvExact(client_fd, &request)) {
                captured_request = request;
                bpx_sdk::SubscribeStateResp response{};
                response.raw[0] = 1;
                response.raw[1] = 2;
                response.raw[2] = 0x34;
                response.raw[3] = 0x12;
                encodeWord(&response.raw, 0, 0x11223344u);
                encodeWord(&response.raw, 1, 0x55667788u);
                send(client_fd, response.raw.data(), response.raw.size(), 0);
            }
            close(client_fd);
        });

        bpx_sdk::TcpSubscribeClient client;
        client.setRobotIp("127.0.0.1");
        client.setTcpLocalPort(kTcpLocalPort);
        client.setSessionId(7);
        client.setRobotStateUploadPort(19873);
        client.setJointStateUploadPort(17895);
        client.setRobotStateUploadRate(250);
        client.setHostServerMode(2);
        if (!client.sendStateQueryRequest()) {
            return fail("sendStateQueryRequest reported failure");
        }

        server.join();
        close(server_fd);

        if (!captured_request || !observed_peer_port) {
            return fail("TCP loopback server did not capture a request");
        }
        if (*observed_peer_port != kTcpLocalPort) {
            return fail("TCP client did not bind the configured local port");
        }
        bpx_sdk::SubscribeStateResp response{};
        if (!client.getLatestResponse(&response) ||
            response.responseType() != 1 ||
            response.statusCode() != 2 ||
            response.reserved() != 0x1234 ||
            response.payloadWord(0) != 0x11223344u ||
            response.payloadWord(1) != 0x55667788u ||
            !response.accepted()) {
            return fail("TCP client did not cache the structured subscribe response");
        }
        if (captured_request->session_id != 7 ||
            captured_request->robot_state_upload_port != 19873 ||
            captured_request->joint_state_upload_port != 17895 ||
            captured_request->robot_state_upload_rate_hz != 250 ||
            captured_request->host_server_mode != 2 ||
            captured_request->request_timestamp_ms == 0) {
            return fail("captured TCP request fields were incorrect");
        }
    }

    {
        int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd < 0 || !bindLoopbackSocket(udp_fd, kMotionCommandPort, SOCK_DGRAM)) {
            return fail("failed to open motion UDP listener");
        }
        timeval timeout{};
        timeout.tv_sec = 1;
        setsockopt(udp_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        bpx_sdk::MotionCommandSender sender;
        sender.setRobotIp("127.0.0.1");
        sender.setVelocityControlFlag(true);
        sender.setControlLock(true);
        sender.setGait(static_cast<int>(bpx_sdk::MotionGait::Running), 4);
        sender.setSubGaitType(5);
        if (!sender.sendVelocity(0.2f, -0.4f, 0.6f)) {
            return fail("sendVelocity reported failure");
        }

        MotionCommandWirePacket motion_packet{};
        if (!recvUdpExact(udp_fd, &motion_packet)) {
            return fail("motion UDP listener did not receive a packet");
        }
        if (motion_packet.seq == 0 ||
            motion_packet.command != static_cast<uint8_t>(bpx_sdk::MotionState::Motion) ||
            motion_packet.gait != static_cast<uint8_t>(bpx_sdk::MotionGait::Running) ||
            !closeEnough(motion_packet.values[0], 0.2f) ||
            !closeEnough(motion_packet.values[1], -0.4f) ||
            !closeEnough(motion_packet.values[2], 0.6f) ||
            motion_packet.velocity_control_enabled != 1 ||
            motion_packet.sub_gait != 5 ||
            (motion_packet.control_flags & 0x3u) != 0x3u) {
            return fail("motion command packet contents were incorrect");
        }

        if (!sender.connect(20)) {
            return fail("motion sender connect failed");
        }
        MotionCommandWirePacket loop_packet{};
        if (!recvUdpExact(udp_fd, &loop_packet)) {
            return fail("motion sender runLoop did not emit a packet");
        }
        sender.disconnect();
        sender.close();
        close(udp_fd);
    }

    {
        int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd < 0 || !bindLoopbackSocket(udp_fd, kJointCommandPort, SOCK_DGRAM)) {
            return fail("failed to open joint UDP listener");
        }
        timeval timeout{};
        timeout.tv_sec = 1;
        setsockopt(udp_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        bpx_sdk::JointCommandSender sender;
        sender.setRobotIp("127.0.0.1");
        if (!sender.open()) {
            return fail("joint sender open failed");
        }

        bpx_sdk::JointCommandPacket command{};
        command.pos[0] = 1.25f;
        command.vel[1] = -0.75f;
        command.tff[2] = 0.5f;
        if (!sender.send(command)) {
            return fail("joint sender send failed");
        }

        bpx_sdk::JointCommandPacket received{};
        if (!recvUdpExact(udp_fd, &received)) {
            return fail("joint UDP listener did not receive a packet");
        }
        sender.close();
        close(udp_fd);

        if (!closeEnough(received.pos[0], 1.25f) ||
            !closeEnough(received.vel[1], -0.75f) ||
            !closeEnough(received.tff[2], 0.5f)) {
            return fail("joint command packet contents were incorrect");
        }
    }

    {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0 || !bindLoopbackSocket(server_fd, kTcpServerPort, SOCK_STREAM)) {
            return fail("failed to open loopback TCP server for robot-state stream");
        }

        const uint16_t robot_state_port = reserveLoopbackUdpPort();
        if (robot_state_port == 0) {
            close(server_fd);
            return fail("failed to reserve robot-state UDP port");
        }

        std::thread server([&] {
            sockaddr_in peer{};
            socklen_t peer_size = sizeof(peer);
            const int client_fd =
                accept(server_fd, reinterpret_cast<sockaddr*>(&peer), &peer_size);
            if (client_fd < 0) {
                return;
            }
            bpx_sdk::SubscribeStateReq request{};
            if (recvExact(client_fd, &request)) {
                bpx_sdk::SubscribeStateResp response{};
                response.raw[0] = 1;
                send(client_fd, response.raw.data(), response.raw.size(), 0);
            }
            close(client_fd);
        });

        bpx_sdk::RequestRobotState state;
        state.setRobotIp("127.0.0.1");
        state.setRobotStateUploadPort(robot_state_port);
        if (!state.connect()) {
            server.join();
            close(server_fd);
            return fail("RequestRobotState connect failed");
        }

        server.join();
        close(server_fd);

        int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd < 0) {
            state.disconnect();
            return fail("failed to open robot-state UDP sender");
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

        if (!sendUploadPacket(udp_fd, robot_state_port, 1001, 0x1000, data1000) ||
            !sendUploadPacket(udp_fd, robot_state_port, 1002, 0x0200, data200) ||
            !sendUploadPacket(udp_fd, robot_state_port, 1003, 0x0050, data50) ||
            !sendUploadPacket(udp_fd, robot_state_port, 1004, 0x0010, data10) ||
            !sendUploadPacket(udp_fd, robot_state_port, 1005, 0x0001, data1)) {
            close(udp_fd);
            state.disconnect();
            return fail("failed to send robot-state upload packets");
        }
        close(udp_fd);

        bool received_state = false;
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
            uint32_t joint_ts = 0;
            uint32_t imu_ts = 0;
            uint32_t odom_ts = 0;
            uint32_t motion_ts = 0;
            uint32_t battery_ts = 0;
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
                state.getJointStateTimestamp(&joint_ts) &&
                state.getImuTimestamp(&imu_ts) &&
                state.getOdometryTimestamp(&odom_ts) &&
                state.getMotionStateTimestamp(&motion_ts) &&
                state.getBatteryTimestamp(&battery_ts) &&
                closeEnough(joint_pos[0], data1000.joint_position[0]) &&
                closeEnough(joint_vel[1], data1000.joint_velocity[1]) &&
                closeEnough(joint_tau[2], data1000.joint_torque[2]) &&
                closeEnough(imu_rpy[2], data200.imu_rpy[2]) &&
                closeEnough(imu_quat[3], data200.imu_quat[3]) &&
                closeEnough(imu_acc[1], data200.imu_acc[1]) &&
                closeEnough(imu_omega[0], data200.imu_omega[0]) &&
                closeEnough(leg_odom.velocity_body[0], data50.leg_odom.velocity_body[0]) &&
                closeEnough(leg_odom.position[1], data50.leg_odom.position[1]) &&
                closeEnough(leg_odom.orientation[3], data50.leg_odom.orientation[3]) &&
                closeEnough(leg_odom.angular_velocity[2], data50.leg_odom.angular_velocity[2]) &&
                closeEnough(motor_temperature[0], 71.0f) &&
                closeEnough(driver_temperature[1], 58.0f) &&
                closeEnough(max_velocity[0], data10.max_velocity[0]) &&
                battery_level == data1.battery_level &&
                closeEnough(battery_current, data1.battery_current) &&
                current_motion_state == data10.current_motion_state &&
                current_gait == data10.current_gait &&
                last_motion_state == data10.last_motion_state &&
                last_gait == data10.last_gait &&
                sub_gait == static_cast<uint8_t>(data10.sub_gait) &&
                joint_ts == 1001 &&
                imu_ts == 1002 &&
                odom_ts == 1003 &&
                motion_ts == 1004 &&
                battery_ts == 1005) {
                received_state = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        state.disconnect();
        if (!received_state) {
            return fail("RequestRobotState did not surface live robot-state UDP updates");
        }
    }

    {
        int server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0 || !bindLoopbackSocket(server_fd, kTcpServerPort, SOCK_STREAM)) {
            return fail("failed to open loopback TCP server for joint feedback");
        }

        const uint16_t robot_state_port = reserveLoopbackUdpPort();
        const uint16_t joint_state_port = reserveLoopbackUdpPort();
        if (robot_state_port == 0 || joint_state_port == 0 || robot_state_port == joint_state_port) {
            close(server_fd);
            return fail("failed to reserve loopback UDP ports");
        }

        std::thread server([&] {
            sockaddr_in peer{};
            socklen_t peer_size = sizeof(peer);
            const int client_fd =
                accept(server_fd, reinterpret_cast<sockaddr*>(&peer), &peer_size);
            if (client_fd < 0) {
                return;
            }
            bpx_sdk::SubscribeStateReq request{};
            if (recvExact(client_fd, &request)) {
                bpx_sdk::SubscribeStateResp response{};
                response.raw[0] = 1;
                send(client_fd, response.raw.data(), response.raw.size(), 0);
            }
            close(client_fd);
        });

        bpx_sdk::JointLevelControl joint;
        joint.setRobotIp("127.0.0.1");
        joint.setRobotStateUploadPort(robot_state_port);
        joint.setJointStateUploadPort(joint_state_port);
        if (!joint.connect()) {
            server.join();
            close(server_fd);
            return fail("JointLevelControl connect failed");
        }

        server.join();
        close(server_fd);

        int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_fd < 0) {
            joint.disconnect();
            return fail("failed to open joint feedback UDP sender");
        }

        sockaddr_in address{};
        std::memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_port = htons(joint_state_port);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

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
        if (sendto(udp_fd, &feedback, sizeof(feedback), 0,
                   reinterpret_cast<const sockaddr*>(&address),
                   sizeof(address)) != static_cast<ssize_t>(sizeof(feedback))) {
            close(udp_fd);
            joint.disconnect();
            return fail("failed to send joint feedback packet");
        }
        close(udp_fd);

        bool received_feedback = false;
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
                received_feedback = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        joint.disconnect();
        if (!received_feedback) {
            return fail("JointLevelControl did not consume live joint feedback");
        }
    }

    return 0;
}

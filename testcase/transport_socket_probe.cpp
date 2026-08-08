#include "../src/joint_command_sender.h"
#include "../src/motion_command_sender.h"
#include "../src/tcp_subscribe_client.h"

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
            motion_packet.command != static_cast<uint8_t>(bpx_sdk::MotionCommand::Velocity) ||
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

    return 0;
}

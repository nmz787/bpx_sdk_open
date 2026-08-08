#include "../include/joint_level_control.h"
#include "../include/request_robot_state.h"
#include "../src/recovery_runtime.h"

#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

constexpr uint16_t kTcpServerPort = 10860;

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

struct SubscribeCapture {
    uint16_t robot_state_upload_port = 0;
    uint16_t joint_state_upload_port = 0;
    uint16_t robot_state_upload_rate_hz = 0;
    uint8_t host_server_mode = 0;
};

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

}  // namespace

int main() {
    SubscribeCapture request_state{};
    const uint16_t robot_state_port = reserveLoopbackUdpPort();
    if (robot_state_port == 0) {
        return 1;
    }
    auto state_server = runSubscribeServer(&request_state);
    if (state_server.first < 0) {
        return 1;
    }

    bpx_sdk::RequestRobotState state;
    state.setRobotIp("127.0.0.1");
    state.setRobotStateUploadPort(robot_state_port);
    if (!state.connect()) {
        close(state_server.first);
        state_server.second.join();
        return 1;
    }
    state.disconnect();
    close(state_server.first);
    state_server.second.join();

    SubscribeCapture request_joint{};
    const uint16_t joint_robot_state_port = reserveLoopbackUdpPort();
    const uint16_t joint_state_port = reserveLoopbackUdpPort();
    if (joint_robot_state_port == 0 || joint_state_port == 0 ||
        joint_robot_state_port == joint_state_port) {
        return 1;
    }
    auto joint_server = runSubscribeServer(&request_joint);
    if (joint_server.first < 0) {
        return 1;
    }

    bpx_sdk::JointLevelControl joint;
    joint.setRobotIp("127.0.0.1");
    joint.setRobotStateUploadPort(joint_robot_state_port);
    joint.setJointStateUploadPort(joint_state_port);
    if (!joint.connect()) {
        close(joint_server.first);
        joint_server.second.join();
        return 1;
    }
    joint.disconnect();
    close(joint_server.first);
    joint_server.second.join();

    printValue("request0.robot_state_port_matches",
               static_cast<int>(request_state.robot_state_upload_port == robot_state_port));
    printValue("request0.joint_state_port_matches",
               static_cast<int>(request_state.joint_state_upload_port ==
                                bpx_sdk::DEFAULT_CLIENT_JOINT_STATE_UDP_PORT));
    printValue("request0.robot_state_upload_rate_hz", request_state.robot_state_upload_rate_hz);
    printValue("request1.robot_state_port_matches",
               static_cast<int>(request_joint.robot_state_upload_port == joint_robot_state_port));
    printValue("request1.joint_state_port_matches",
               static_cast<int>(request_joint.joint_state_upload_port == joint_state_port));
    printValue("request1.robot_state_upload_rate_hz", request_joint.robot_state_upload_rate_hz);
    return 0;
}

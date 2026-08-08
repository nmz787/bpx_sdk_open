#include "tcp_subscribe_client.h"

#include "bpx_sdk_config.h"
#include "robot_state_udp_receiver.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bpx_sdk {
namespace {

constexpr uint16_t kDefaultTcpServerPort = 10860;

bool decodeAccepted(const SubscribeStateResp& response) {
    return response.raw[0] != 0;
}

void closeSocketFd(int* fd) {
    if (!fd || *fd < 0) {
        return;
    }
    close(*fd);
    *fd = -1;
}

bool setReuseAddr(int fd) {
    const int enabled = 1;
    return setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) == 0;
}

bool fillSockaddr(const char* ip, uint16_t port, sockaddr_in* address) {
    if (!ip || !address) {
        return false;
    }
    std::memset(address, 0, sizeof(*address));
    address->sin_family = AF_INET;
    address->sin_port = htons(port);
    return inet_pton(AF_INET, ip, &address->sin_addr) == 1;
}

bool receiveResponseBestEffort(int fd, SubscribeStateResp* response) {
    if (fd < 0 || !response) {
        return false;
    }
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    unsigned char* buffer = response->raw.data();
    size_t received = 0;
    while (received < response->raw.size()) {
        const ssize_t chunk = recv(fd, buffer + received, response->raw.size() - received, 0);
        if (chunk <= 0) {
            return false;
        }
        received += static_cast<size_t>(chunk);
    }
    return true;
}

}  // namespace

TcpSubscribeClient::TcpSubscribeClient()
    : robot_ip_(DEFAULT_SERVER_IP),
      server_port_(kDefaultTcpServerPort) {}

TcpSubscribeClient::~TcpSubscribeClient() = default;

void TcpSubscribeClient::disconnect() {
    closeSocketFd(&response_socket_fd_);
    response_loop_running_ = false;
}

void TcpSubscribeClient::setSessionId(uint16_t session_id) {
    session_id_ = session_id;
}

void TcpSubscribeClient::setTcpLocalPort(uint16_t local_port) {
    tcp_local_port_ = local_port;
}

bool TcpSubscribeClient::startStateQuery() {
    SubscribeStateReq request;
    request.session_id = session_id_;
    request.robot_state_upload_port = robot_state_upload_port_;
    request.joint_state_upload_port = joint_state_upload_port_;
    request.robot_state_upload_rate_hz = robot_state_upload_rate_hz_;
    request.host_server_mode = host_server_mode_;
    if (!sendRequest(request)) {
        return false;
    }
    if (receiver_) {
        receiver_->storeLatest(makeConnectedSnapshot());
    }
    return true;
}

bool TcpSubscribeClient::queryRobotVersion(RobotVersionInfo* info) {
    if (!info) {
        return false;
    }
    *info = currentSdkVersion();
    return true;
}

void TcpSubscribeClient::setHostServerMode(unsigned char host_server_mode) {
    host_server_mode_ = host_server_mode;
}

void TcpSubscribeClient::setJointStateUploadPort(uint16_t upload_port) {
    joint_state_upload_port_ = upload_port;
}

void TcpSubscribeClient::setRobotStateUploadPort(uint16_t upload_port) {
    robot_state_upload_port_ = upload_port;
}

void TcpSubscribeClient::setRobotStateUploadRate(uint16_t upload_rate_hz) {
    robot_state_upload_rate_hz_ = upload_rate_hz;
}

void TcpSubscribeClient::setRobotIp(const char* ip) {
    if (!ip || ip[0] == '\0') {
        return;
    }
    robot_ip_ = ip;
}

bool TcpSubscribeClient::sendRequest(const SubscribeStateReq& request) const {
    SubscribeStateReq wire = request;
    wire.request_timestamp_ms = nowMs();

    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        return true;
    }

    if (!bindLocalTcpPort(socket_fd)) {
        closeSocketFd(&socket_fd);
        return true;
    }

    sockaddr_in server_address{};
    if (!fillSockaddr(robot_ip_.c_str(), server_port_, &server_address)) {
        closeSocketFd(&socket_fd);
        return true;
    }

    if (connect(socket_fd, reinterpret_cast<const sockaddr*>(&server_address),
                sizeof(server_address)) != 0) {
        closeSocketFd(&socket_fd);
        return true;
    }

    if (!sendAll(socket_fd, reinterpret_cast<const unsigned char*>(&wire), sizeof(wire))) {
        closeSocketFd(&socket_fd);
        return true;
    }

    SubscribeStateResp response{};
    if (receiveResponseBestEffort(socket_fd, &response)) {
        printResponse(response);
    }
    closeSocketFd(&socket_fd);
    return true;
}

void TcpSubscribeClient::printResponse(const SubscribeStateResp& response) const {
    (void)decodeAccepted(response);
}

bool TcpSubscribeClient::bindLocalTcpPort(int socket_fd) const {
    if (tcp_local_port_ == 0) {
        return true;
    }
    if (!setReuseAddr(socket_fd)) {
        return false;
    }
    sockaddr_in local_address{};
    std::memset(&local_address, 0, sizeof(local_address));
    local_address.sin_family = AF_INET;
    local_address.sin_addr.s_addr = htonl(INADDR_ANY);
    local_address.sin_port = htons(tcp_local_port_);
    return bind(socket_fd, reinterpret_cast<const sockaddr*>(&local_address),
                sizeof(local_address)) == 0;
}

bool TcpSubscribeClient::sendDefaultRequest() const {
    SubscribeStateReq request;
    request.session_id = session_id_;
    request.robot_state_upload_port = robot_state_upload_port_;
    request.joint_state_upload_port = joint_state_upload_port_;
    request.robot_state_upload_rate_hz = robot_state_upload_rate_hz_;
    request.host_server_mode = host_server_mode_;
    return sendRequest(request);
}

bool TcpSubscribeClient::sendStateQueryRequest() const {
    return sendDefaultRequest();
}

uint32_t TcpSubscribeClient::nowMs() const {
    return bpx_sdk::nowMs();
}

bool TcpSubscribeClient::recvAll(int socket_fd, unsigned char* buffer, unsigned long size) const {
    if (socket_fd < 0 || !buffer) {
        return false;
    }
    unsigned long received = 0;
    while (received < size) {
        const ssize_t chunk = recv(socket_fd, buffer + received, size - received, 0);
        if (chunk <= 0) {
            return false;
        }
        received += static_cast<unsigned long>(chunk);
    }
    return true;
}

bool TcpSubscribeClient::sendAll(int socket_fd, const unsigned char* buffer, unsigned long size) const {
    if (socket_fd < 0 || !buffer) {
        return false;
    }
    unsigned long sent = 0;
    while (sent < size) {
        const ssize_t chunk = send(socket_fd, buffer + sent, size - sent, 0);
        if (chunk <= 0) {
            return false;
        }
        sent += static_cast<unsigned long>(chunk);
    }
    return true;
}

void TcpSubscribeClient::attachReceiver(RobotStateUdpReceiver* receiver) {
    receiver_ = receiver;
}

}  // namespace bpx_sdk

#include "tcp_subscribe_client.h"

#include "bpx_sdk_config.h"
#include "robot_state_udp_receiver.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bpx_sdk {
namespace {

constexpr uint16_t kDefaultTcpServerPort = 10860;

bool decodeAccepted(const SubscribeStateResp& response) {
    return response.accepted();
}

void closeSocketFd(int* fd) {
    if (!fd || *fd < 0) {
        return;
    }
    close(*fd);
    *fd = -1;
}

void shutdownSocketFd(int fd) {
    if (fd >= 0) {
        shutdown(fd, SHUT_RDWR);
    }
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

bool connectWithTimeout(int fd, const sockaddr_in& address) {
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
        return false;
    }

    const int result =
        connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address));
    if (result == 0) {
        fcntl(fd, F_SETFL, flags);
        return true;
    }
    if (errno != EINPROGRESS) {
        fcntl(fd, F_SETFL, flags);
        return false;
    }

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(fd, &write_fds);
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    const int ready = select(fd + 1, nullptr, &write_fds, nullptr, &timeout);
    if (ready <= 0) {
        fcntl(fd, F_SETFL, flags);
        return false;
    }

    int socket_error = 0;
    socklen_t socket_error_size = sizeof(socket_error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_size) != 0 ||
        socket_error != 0) {
        fcntl(fd, F_SETFL, flags);
        return false;
    }

    fcntl(fd, F_SETFL, flags);
    return true;
}

}  // namespace

TcpSubscribeClient::TcpSubscribeClient()
    : robot_ip_(DEFAULT_SERVER_IP),
      server_port_(kDefaultTcpServerPort) {}

TcpSubscribeClient::~TcpSubscribeClient() {
    disconnect();
}

void TcpSubscribeClient::disconnect() {
    response_loop_running_ = false;
    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        shutdownSocketFd(response_socket_fd_);
    }
    if (response_thread_.joinable()) {
        response_thread_.join();
    }
    clearLatestResponse();
}

void TcpSubscribeClient::setSessionId(uint16_t session_id) {
    session_id_ = session_id;
}

void TcpSubscribeClient::setTcpLocalPort(uint16_t local_port) {
    tcp_local_port_ = local_port;
}

bool TcpSubscribeClient::startStateQuery() {
    if (response_loop_running_.exchange(true)) {
        return true;
    }

    if (response_thread_.joinable()) {
        response_thread_.join();
    }

    SubscribeStateReq request;
    request.session_id = session_id_;
    request.robot_state_upload_port = robot_state_upload_port_;
    request.joint_state_upload_port = joint_state_upload_port_;
    request.robot_state_upload_rate_hz = robot_state_upload_rate_hz_;
    request.host_server_mode = host_server_mode_;
    request.request_timestamp_ms = nowMs();

    if (robot_ip_ != DEFAULT_SERVER_IP) {
        int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd < 0) {
            response_loop_running_ = false;
            return false;
        }

        if (!bindLocalTcpPort(socket_fd)) {
            closeSocketFd(&socket_fd);
            response_loop_running_ = false;
            return false;
        }

        sockaddr_in server_address{};
        if (!fillSockaddr(robot_ip_.c_str(), server_port_, &server_address)) {
            closeSocketFd(&socket_fd);
            response_loop_running_ = false;
            return false;
        }

        if (!connectWithTimeout(socket_fd, server_address)) {
            closeSocketFd(&socket_fd);
            response_loop_running_ = false;
            return false;
        }

        if (!sendAll(socket_fd, reinterpret_cast<const unsigned char*>(&request),
                     sizeof(request))) {
            closeSocketFd(&socket_fd);
            response_loop_running_ = false;
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(response_mutex_);
            response_socket_fd_ = socket_fd;
        }

        response_thread_ = std::thread([this, socket_fd]() {
            while (response_loop_running_) {
                SubscribeStateResp response{};
                if (!recvAll(socket_fd, response.raw.data(),
                             response.raw.size())) {
                    break;
                }
                printResponse(response);
            }
            close(socket_fd);
            {
                std::lock_guard<std::mutex> lock(response_mutex_);
                if (response_socket_fd_ == socket_fd) {
                    response_socket_fd_ = -1;
                }
            }
            response_loop_running_ = false;
        });
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

    if (robot_ip_ == DEFAULT_SERVER_IP) {
        return true;
    }

    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        return false;
    }

    if (!bindLocalTcpPort(socket_fd)) {
        closeSocketFd(&socket_fd);
        return false;
    }

    sockaddr_in server_address{};
    if (!fillSockaddr(robot_ip_.c_str(), server_port_, &server_address)) {
        closeSocketFd(&socket_fd);
        return false;
    }

    if (!connectWithTimeout(socket_fd, server_address)) {
        closeSocketFd(&socket_fd);
        return false;
    }

    if (!sendAll(socket_fd, reinterpret_cast<const unsigned char*>(&wire), sizeof(wire))) {
        closeSocketFd(&socket_fd);
        return false;
    }

    SubscribeStateResp response{};
    if (receiveResponseBestEffort(socket_fd, &response)) {
        storeLatestResponse(response);
        printResponse(response);
    }
    closeSocketFd(&socket_fd);
    return true;
}

void TcpSubscribeClient::printResponse(const SubscribeStateResp& response) const {
    (void)decodeAccepted(response);
}

bool TcpSubscribeClient::getLatestResponse(SubscribeStateResp* response) const {
    if (!response) {
        return false;
    }
    std::lock_guard<std::mutex> lock(response_mutex_);
    if (!has_latest_response_) {
        return false;
    }
    *response = latest_response_;
    return true;
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

void TcpSubscribeClient::clearLatestResponse() const {
    std::lock_guard<std::mutex> lock(response_mutex_);
    latest_response_ = SubscribeStateResp{};
    has_latest_response_ = false;
}

void TcpSubscribeClient::storeLatestResponse(const SubscribeStateResp& response) const {
    std::lock_guard<std::mutex> lock(response_mutex_);
    latest_response_ = response;
    has_latest_response_ = true;
}

}  // namespace bpx_sdk

#include "joint_state_receiver.h"

#include <arpa/inet.h>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bpx_sdk {
namespace {

void closeSocketFd(int* fd) {
    if (!fd || *fd < 0) {
        return;
    }
    close(*fd);
    *fd = -1;
}

bool setNonBlocking(int fd) {
    const int flags = fcntl(fd, F_GETFL, 0);
    return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

}  // namespace

JointStateReceiver::JointStateReceiver(uint16_t listen_port)
    : listen_port_(listen_port) {}

JointStateReceiver::~JointStateReceiver() {
    close();
}

void JointStateReceiver::receiveLoop() {
    while (running_) {
        JointStatePacket packet{};
        if (receiveOnce(&packet)) {
            storeLatest(packet);
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool JointStateReceiver::receiveOnce(JointStatePacket* packet) const {
    if (!packet || !const_cast<JointStateReceiver*>(this)->open()) {
        return false;
    }

    sockaddr_in peer{};
    socklen_t peer_size = sizeof(peer);
    const ssize_t received =
        recvfrom(socket_fd_, packet, sizeof(*packet), 0,
                 reinterpret_cast<sockaddr*>(&peer), &peer_size);
    if (received < 0) {
        return false;
    }
    return received == static_cast<ssize_t>(sizeof(*packet));
}

void JointStateReceiver::storeLatest(const JointStatePacket& packet) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_packet_ = packet;
    has_packet_ = true;
}

void JointStateReceiver::setListenPort(unsigned short listen_port) {
    listen_port_ = listen_port;
}

void JointStateReceiver::run() {
    if (!open()) {
        return;
    }
    receiveLoop();
}

bool JointStateReceiver::open() {
    if (socket_fd_ >= 0) {
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
        closeSocketFd(&socket_fd_);
        return false;
    }

    if (!setNonBlocking(socket_fd_)) {
        closeSocketFd(&socket_fd_);
        return false;
    }
    return true;
}

void JointStateReceiver::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
}

void JointStateReceiver::close() {
    stop();
    closeSocketFd(&socket_fd_);
}

void JointStateReceiver::start() {
    if (!open()) {
        return;
    }
    if (running_.exchange(true)) {
        return;
    }
    receive_thread_ = std::thread(&JointStateReceiver::receiveLoop, this);
}

bool JointStateReceiver::getLatest(JointStatePacket* packet) const {
    if (!packet) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_packet_) {
        return false;
    }
    *packet = latest_packet_;
    return true;
}

}  // namespace bpx_sdk

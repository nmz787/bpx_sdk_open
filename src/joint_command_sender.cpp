#include "joint_command_sender.h"

#include "bpx_sdk_config.h"
#include "joint_state_receiver.h"

#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bpx_sdk {
namespace {

constexpr uint16_t kJointCommandPort = 7896;

bool fillSockaddr(const char* ip, uint16_t port, sockaddr_in* address) {
    if (!ip || !address) {
        return false;
    }
    std::memset(address, 0, sizeof(*address));
    address->sin_family = AF_INET;
    address->sin_port = htons(port);
    return inet_pton(AF_INET, ip, &address->sin_addr) == 1;
}

void closeSocketFd(int* fd) {
    if (!fd || *fd < 0) {
        return;
    }
    close(*fd);
    *fd = -1;
}

}  // namespace

JointCommandSender::JointCommandSender()
    : robot_ip_(DEFAULT_SERVER_IP) {}

JointCommandSender::~JointCommandSender() = default;

bool JointCommandSender::open() {
    if (socket_fd_ >= 0) {
        return true;
    }
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    return socket_fd_ >= 0;
}

bool JointCommandSender::send(const JointCommandPacket& packet) {
    latest_command_ = packet;

    bool sent = false;
    if (robot_ip_ != DEFAULT_SERVER_IP && open()) {
        sockaddr_in address{};
        if (fillSockaddr(robot_ip_.c_str(), kJointCommandPort, &address)) {
            const ssize_t bytes = sendto(socket_fd_, &packet, sizeof(packet), 0,
                                         reinterpret_cast<const sockaddr*>(&address),
                                         sizeof(address));
            sent = bytes == static_cast<ssize_t>(sizeof(packet));
        }
    }

    if (receiver_ && robot_ip_ == DEFAULT_SERVER_IP) {
        JointStatePacket state_packet;
        state_packet.joint_position = packet.pos;
        state_packet.joint_velocity = packet.vel;
        state_packet.joint_torque = packet.tff;
        state_packet.timestamp_ms = static_cast<float>(nowMs());
        state_packet.seq = ++seq_;
        receiver_->storeLatest(state_packet);
    }
    return sent || robot_ip_ == DEFAULT_SERVER_IP;
}

void JointCommandSender::close() {
    closeSocketFd(&socket_fd_);
}

bool JointCommandSender::sendZero() {
    JointCommandPacket packet;
    return send(packet);
}

void JointCommandSender::setRobotIp(const char* ip) {
    if (!ip || ip[0] == '\0') {
        return;
    }
    close();
    robot_ip_ = ip;
}

void JointCommandSender::attachReceiver(JointStateReceiver* receiver) {
    receiver_ = receiver;
}

bool JointCommandSender::isOpen() const {
    return socket_fd_ >= 0;
}

}  // namespace bpx_sdk

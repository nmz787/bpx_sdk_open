#include "motion_command_sender.h"

#include "bpx_sdk_config.h"
#include "robot_state_udp_receiver.h"

#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace bpx_sdk {
namespace {

constexpr uint16_t kMotionCommandPort = 9527;

struct MotionCommandWirePacket {
    uint32_t seq = 0;
    uint8_t command = static_cast<uint8_t>(MotionState::Motion);
    uint8_t gait = static_cast<uint8_t>(MotionGait::Walk);
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

static_assert(sizeof(MotionCommandWirePacket) == 56,
              "Motion command packets must match the recovered 56-byte wire layout");

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

MotionCommand motionCommandFromState(uint8_t command) {
    switch (command) {
        case static_cast<uint8_t>(MotionState::StandingUp):
            return MotionCommand::StandUp;
        case static_cast<uint8_t>(MotionState::Passive):
            return MotionCommand::Damping;
        case static_cast<uint8_t>(MotionState::SitDown):
            return MotionCommand::SitDown;
        case static_cast<uint8_t>(MotionState::Motion):
            return MotionCommand::Velocity;
        default:
            return MotionCommand::None;
    }
}

uint8_t wireCommandFromMotionCommand(MotionCommand command) {
    switch (command) {
        case MotionCommand::StandUp:
            return static_cast<uint8_t>(MotionState::StandingUp);
        case MotionCommand::SitDown:
            return static_cast<uint8_t>(MotionState::SitDown);
        case MotionCommand::Damping:
            return static_cast<uint8_t>(MotionState::Passive);
        case MotionCommand::Velocity:
            return static_cast<uint8_t>(MotionState::Motion);
        case MotionCommand::None:
        default:
            return 0;
    }
}

}  // namespace

MotionCommandSender::MotionCommandSender()
    : robot_ip_(DEFAULT_SERVER_IP) {}

MotionCommandSender::~MotionCommandSender() {
    disconnect();
    close();
}

void MotionCommandSender::disconnect() {
    connected_ = false;
    if (loop_thread_.joinable()) {
        loop_thread_.join();
    }
}

bool MotionCommandSender::sendLatest() {
    return sendPacket(motionCommandFromState(command_));
}

bool MotionCommandSender::sendPacket(MotionCommand command) {
    MotionCommandWirePacket packet;
    packet.seq = static_cast<uint32_t>(seq_.fetch_add(1) + 1);
    packet.command = wireCommandFromMotionCommand(command);
    uint8_t gait = 0;
    int8_t sub_gait = 0;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        gait = gait_;
        sub_gait = sub_gait_;
        packet.gait = gait;
        packet.values = command_values_;
        packet.velocity_control_enabled = velocity_control_enabled_;
        packet.zero_positions_nonce = zero_positions_nonce_;
        packet.sub_gait = sub_gait;
        packet.reserved2 = reserved_;
        packet.control_flags = control_flags_;
        packet.reserved_tail = reserved_tail_;
    }

    bool sent = false;
    if (robot_ip_ != DEFAULT_SERVER_IP && open()) {
        sockaddr_in address{};
        if (fillSockaddr(robot_ip_.c_str(), kMotionCommandPort, &address)) {
            const ssize_t bytes = sendto(socket_fd_, &packet, sizeof(packet), 0,
                                         reinterpret_cast<const sockaddr*>(&address),
                                         sizeof(address));
            sent = bytes == static_cast<ssize_t>(sizeof(packet));
        }
    }

    if (receiver_) {
        RobotStateSnapshot snapshot;
        if (!receiver_->getLatestState(&snapshot)) {
            snapshot = makeConnectedSnapshot();
        }
        switch (command) {
            case MotionCommand::StandUp:
                applyMotionState(&snapshot, MotionState::StandingUp);
                break;
            case MotionCommand::SitDown:
                applyMotionState(&snapshot, MotionState::SitDown);
                break;
            case MotionCommand::Damping:
                applyMotionState(&snapshot, MotionState::Passive);
                break;
            case MotionCommand::Velocity:
                applyMotionState(&snapshot, MotionState::Motion);
                break;
            case MotionCommand::None:
                break;
        }
        applyGaitSelection(&snapshot, static_cast<MotionGait>(gait), sub_gait);
        receiver_->storeLatest(snapshot);
    }

    return sent || robot_ip_ == DEFAULT_SERVER_IP;
}

bool MotionCommandSender::sendDamping() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        command_ = static_cast<uint8_t>(MotionState::Passive);
        gait_ = static_cast<uint8_t>(MotionGait::Walk);
        sub_gait_ = 0;
        command_values_.fill(0.0f);
    }
    return sendPacket(MotionCommand::Damping);
}

bool MotionCommandSender::sendSitDown() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        command_ = static_cast<uint8_t>(MotionState::SitDown);
        gait_ = static_cast<uint8_t>(MotionGait::Walk);
        sub_gait_ = 0;
        command_values_.fill(0.0f);
    }
    return sendPacket(MotionCommand::SitDown);
}

bool MotionCommandSender::sendStandUp() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        command_ = static_cast<uint8_t>(MotionState::StandingUp);
        gait_ = static_cast<uint8_t>(MotionGait::Walk);
        sub_gait_ = 0;
        command_values_.fill(0.0f);
    }
    return sendPacket(MotionCommand::StandUp);
}

bool MotionCommandSender::sendVelocity(float x, float y, float yaw) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        command_ = static_cast<uint8_t>(MotionState::Motion);
        command_values_.fill(0.0f);
        command_values_[0] = x;
        command_values_[1] = y;
        command_values_[2] = yaw;
    }
    return sendPacket(MotionCommand::Velocity);
}

void MotionCommandSender::setControlLock(bool locked) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    control_flags_ |= 0x1u;
    if (locked) {
        control_flags_ |= 0x2u;
    } else {
        control_flags_ &= ~0x2u;
    }
}

void MotionCommandSender::setSubGaitType(unsigned char sub_gait) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sub_gait_ = sub_gait;
}

void MotionCommandSender::setZeroPositionsFlag() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++zero_positions_nonce_;
    if (zero_positions_nonce_ == 0) {
        ++zero_positions_nonce_;
    }
}

void MotionCommandSender::setVelocityControlFlag(bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    velocity_control_enabled_ = enabled ? 1 : 0;
}

bool MotionCommandSender::open() {
    if (socket_fd_ >= 0) {
        return true;
    }
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    return socket_fd_ >= 0;
}

void MotionCommandSender::close() {
    closeSocketFd(&socket_fd_);
}

bool MotionCommandSender::connect(unsigned short rate_hz) {
    if (rate_hz == 0) {
        return false;
    }
    if (!open()) {
        return false;
    }
    if (connected_.exchange(true)) {
        return true;
    }
    loop_thread_ = std::thread(&MotionCommandSender::runLoop, this, rate_hz);
    return true;
}

void MotionCommandSender::runLoop(unsigned short rate_hz) {
    const auto sleep_time =
        std::chrono::microseconds(1000000 / static_cast<int>(rate_hz));
    while (connected_) {
        sendLatest();
        std::this_thread::sleep_for(sleep_time);
    }
}

void MotionCommandSender::setGait(int gait, unsigned char sub_gait) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    gait_ = static_cast<uint8_t>(gait);
    sub_gait_ = sub_gait;
}

void MotionCommandSender::setRobotIp(const char* ip) {
    if (!ip || ip[0] == '\0') {
        return;
    }
    disconnect();
    close();
    robot_ip_ = ip;
}

void MotionCommandSender::attachReceiver(RobotStateUdpReceiver* receiver) {
    receiver_ = receiver;
}

bool MotionCommandSender::isConnected() const {
    return connected_;
}

}  // namespace bpx_sdk

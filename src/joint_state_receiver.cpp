#include "joint_state_receiver.h"

namespace bpx_sdk {

JointStateReceiver::JointStateReceiver(uint16_t listen_port)
    : listen_port_(listen_port) {}

JointStateReceiver::~JointStateReceiver() = default;

void JointStateReceiver::receiveLoop() {}

bool JointStateReceiver::receiveOnce(JointStatePacket* packet) const {
    return getLatest(packet);
}

void JointStateReceiver::storeLatest(const JointStatePacket& packet) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_packet_ = packet;
    has_packet_ = true;
}

void JointStateReceiver::setListenPort(unsigned short listen_port) {
    listen_port_ = listen_port;
}

void JointStateReceiver::run() {}

bool JointStateReceiver::open() {
    running_ = true;
    return true;
}

void JointStateReceiver::stop() {
    running_ = false;
}

void JointStateReceiver::close() {
    running_ = false;
}

void JointStateReceiver::start() {
    running_ = true;
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

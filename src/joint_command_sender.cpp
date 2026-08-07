#include "joint_command_sender.h"

#include "joint_state_receiver.h"

namespace bpx_sdk {

JointCommandSender::JointCommandSender() = default;
JointCommandSender::~JointCommandSender() = default;

bool JointCommandSender::open() {
    open_ = true;
    return true;
}

bool JointCommandSender::send(const JointCommandPacket& packet) {
    latest_command_ = packet;
    if (!open_) {
        return false;
    }
    if (receiver_) {
        JointStatePacket state_packet;
        state_packet.joint_position = packet.pos;
        state_packet.joint_velocity = packet.vel;
        state_packet.joint_torque = packet.tff;
        state_packet.timestamp_ms = static_cast<float>(nowMs());
        state_packet.seq = ++seq_;
        receiver_->storeLatest(state_packet);
    }
    return true;
}

void JointCommandSender::close() {
    open_ = false;
}

bool JointCommandSender::sendZero() {
    JointCommandPacket packet;
    return send(packet);
}

void JointCommandSender::attachReceiver(JointStateReceiver* receiver) {
    receiver_ = receiver;
}

bool JointCommandSender::isOpen() const {
    return open_;
}

}  // namespace bpx_sdk

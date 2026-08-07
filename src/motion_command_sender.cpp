#include "motion_command_sender.h"

#include "robot_state_udp_receiver.h"

namespace bpx_sdk {

MotionCommandSender::MotionCommandSender() = default;
MotionCommandSender::~MotionCommandSender() = default;

void MotionCommandSender::disconnect() {
    close();
}

bool MotionCommandSender::sendLatest() {
    if (!connected_ || !receiver_) {
        return connected_;
    }
    RobotStateSnapshot snapshot;
    if (!receiver_->getLatestState(&snapshot)) {
        snapshot = makeConnectedSnapshot();
    }
    applyGaitSelection(&snapshot, gait_, sub_gait_);
    if (velocity_control_enabled_) {
        applyMotionState(&snapshot, MotionState::Motion);
    }
    receiver_->storeLatest(snapshot);
    return true;
}

bool MotionCommandSender::sendPacket(MotionCommand command) {
    if (!connected_ || !receiver_) {
        return connected_;
    }
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
    applyGaitSelection(&snapshot, gait_, sub_gait_);
    receiver_->storeLatest(snapshot);
    return true;
}

bool MotionCommandSender::sendDamping() {
    return sendPacket(MotionCommand::Damping);
}

bool MotionCommandSender::sendSitDown() {
    return sendPacket(MotionCommand::SitDown);
}

bool MotionCommandSender::sendStandUp() {
    return sendPacket(MotionCommand::StandUp);
}

bool MotionCommandSender::sendVelocity(float x, float y, float yaw) {
    velocity_x_ = x;
    velocity_y_ = y;
    velocity_yaw_ = yaw;
    return sendPacket(MotionCommand::Velocity);
}

void MotionCommandSender::setControlLock(bool locked) {
    control_lock_ = locked;
}

void MotionCommandSender::setSubGaitType(unsigned char sub_gait) {
    sub_gait_ = static_cast<int8_t>(sub_gait);
}

void MotionCommandSender::setZeroPositionsFlag() {
    zero_positions_flag_ = true;
}

void MotionCommandSender::setVelocityControlFlag(bool enabled) {
    velocity_control_enabled_ = enabled;
}

bool MotionCommandSender::open() {
    connected_ = true;
    return true;
}

void MotionCommandSender::close() {
    connected_ = false;
}

bool MotionCommandSender::connect(unsigned short rate_hz) {
    rate_hz_ = rate_hz;
    return open();
}

void MotionCommandSender::runLoop(unsigned short rate_hz) {
    rate_hz_ = rate_hz;
}

void MotionCommandSender::setGait(int gait, unsigned char sub_gait) {
    gait_ = static_cast<MotionGait>(gait);
    sub_gait_ = static_cast<int8_t>(sub_gait);
}

void MotionCommandSender::attachReceiver(RobotStateUdpReceiver* receiver) {
    receiver_ = receiver;
}

bool MotionCommandSender::isConnected() const {
    return connected_;
}

}  // namespace bpx_sdk

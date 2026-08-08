#include "joint_level_control.h"

#include "joint_command_sender.h"
#include "joint_state_receiver.h"

#include <algorithm>
#include <memory>
#include <optional>

namespace bpx_sdk {

class JointLevelControl::Impl {
public:
    std::array<float, 12> kp{};
    std::array<float, 12> pos{};
    std::array<float, 12> kd{};
    std::array<float, 12> vel{};
    std::array<float, 12> tff{};
    std::array<float, 12> observed_joint_pos{};
    std::array<float, 12> observed_joint_vel{};
    std::array<float, 12> observed_joint_tau{};
    std::array<float, 3> observed_imu_rpy{};
    std::array<float, 4> observed_imu_quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> observed_imu_acc{};
    std::array<float, 3> observed_imu_omega{};
    std::optional<float> high_rate_joint_timestamp;
    std::optional<uint32_t> high_rate_joint_seq;
    std::unique_ptr<JointCommandSender> sender;
    std::unique_ptr<JointStateReceiver> receiver;

    void refreshHighRateFeedback() {
        if (!receiver) {
            return;
        }
        JointStatePacket packet;
        if (!receiver->getLatest(&packet)) {
            return;
        }
        observed_joint_pos = packet.joint_position;
        observed_joint_vel = packet.joint_velocity;
        observed_joint_tau = packet.joint_torque;
        observed_imu_rpy = packet.imu_rpy;
        observed_imu_quat = packet.imu_quat;
        observed_imu_acc = packet.imu_acc;
        observed_imu_omega = packet.imu_omega;
        high_rate_joint_timestamp = packet.timestamp_ms;
        high_rate_joint_seq = packet.seq;
    }
};

JointLevelControl::JointLevelControl()
    : impl_(std::make_unique<Impl>()) {}

JointLevelControl::~JointLevelControl() = default;

bool JointLevelControl::connect() {
    if (!RequestRobotState::connect()) {
        return false;
    }
    if (!impl_->receiver) {
        impl_->receiver = std::make_unique<JointStateReceiver>(jointStateUploadPort());
    }
    impl_->receiver->setListenPort(jointStateUploadPort());
    impl_->receiver->start();
    if (!impl_->sender) {
        impl_->sender = std::make_unique<JointCommandSender>();
    }
    impl_->sender->setRobotIp(robotIp());
    impl_->sender->attachReceiver(impl_->receiver.get());
    return impl_->sender->open();
}

void JointLevelControl::disconnect() {
    if (impl_->sender) {
        impl_->sender->close();
    }
    if (impl_->receiver) {
        impl_->receiver->stop();
        impl_->receiver->close();
    }
    RequestRobotState::disconnect();
}

void JointLevelControl::setJointStateUploadPort(uint16_t port) {
    RequestRobotState::setJointStateUploadPort(port);
    if (impl_->receiver) {
        impl_->receiver->setListenPort(port);
    }
}

bool JointLevelControl::setJointCommand(const std::array<float, 12>& kp,
                                        const std::array<float, 12>& pos,
                                        const std::array<float, 12>& kd,
                                        const std::array<float, 12>& vel,
                                        const std::array<float, 12>& tff) {
    impl_->kp = kp;
    impl_->pos = pos;
    impl_->kd = kd;
    impl_->vel = vel;
    impl_->tff = tff;
    if (impl_->sender && impl_->sender->isOpen()) {
        JointCommandPacket packet;
        packet.kp = kp;
        packet.pos = pos;
        packet.kd = kd;
        packet.vel = vel;
        packet.tff = tff;
        impl_->sender->send(packet);
        impl_->refreshHighRateFeedback();
        return true;
    }
    return false;
}

bool JointLevelControl::setJointKp(const std::array<float, 12>& kp) { impl_->kp = kp; return true; }
bool JointLevelControl::setJointPosition(const std::array<float, 12>& pos) {
    impl_->pos = pos;
    if (impl_->sender && impl_->sender->isOpen()) {
        JointCommandPacket packet{impl_->kp, impl_->pos, impl_->kd, impl_->vel, impl_->tff};
        impl_->sender->send(packet);
        impl_->refreshHighRateFeedback();
        return true;
    }
    return false;
}
bool JointLevelControl::setJointKd(const std::array<float, 12>& kd) { impl_->kd = kd; return true; }
bool JointLevelControl::setJointVelocity(const std::array<float, 12>& vel) {
    impl_->vel = vel;
    if (impl_->sender && impl_->sender->isOpen()) {
        JointCommandPacket packet{impl_->kp, impl_->pos, impl_->kd, impl_->vel, impl_->tff};
        impl_->sender->send(packet);
        impl_->refreshHighRateFeedback();
        return true;
    }
    return false;
}
bool JointLevelControl::setJointTorqueFeedForward(const std::array<float, 12>& tff) {
    impl_->tff = tff;
    if (impl_->sender && impl_->sender->isOpen()) {
        JointCommandPacket packet{impl_->kp, impl_->pos, impl_->kd, impl_->vel, impl_->tff};
        impl_->sender->send(packet);
        impl_->refreshHighRateFeedback();
        return true;
    }
    return false;
}

bool JointLevelControl::setZeroJointCommand() {
    impl_->kp.fill(0.0f);
    impl_->pos.fill(0.0f);
    impl_->kd.fill(0.0f);
    impl_->vel.fill(0.0f);
    impl_->tff.fill(0.0f);
    if (impl_->sender && impl_->sender->isOpen()) {
        impl_->sender->sendZero();
        impl_->refreshHighRateFeedback();
        return true;
    }
    return false;
}

bool JointLevelControl::getJointPositionHighRate(float joint_pos[12]) const {
    impl_->refreshHighRateFeedback();
    if (!joint_pos || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_joint_pos.begin(), impl_->observed_joint_pos.end(), joint_pos);
    return true;
}

bool JointLevelControl::getJointVelocityHighRate(float joint_vel[12]) const {
    impl_->refreshHighRateFeedback();
    if (!joint_vel || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_joint_vel.begin(), impl_->observed_joint_vel.end(), joint_vel);
    return true;
}

bool JointLevelControl::getJointTorqueHighRate(float joint_tau[12]) const {
    impl_->refreshHighRateFeedback();
    if (!joint_tau || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_joint_tau.begin(), impl_->observed_joint_tau.end(), joint_tau);
    return true;
}

bool JointLevelControl::getImuRpyHighRate(float rpy[3]) const {
    impl_->refreshHighRateFeedback();
    if (!rpy || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_imu_rpy.begin(), impl_->observed_imu_rpy.end(), rpy);
    return true;
}

bool JointLevelControl::getImuQuatHighRate(float quat[4]) const {
    impl_->refreshHighRateFeedback();
    if (!quat || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_imu_quat.begin(), impl_->observed_imu_quat.end(), quat);
    return true;
}

bool JointLevelControl::getImuAccHighRate(float acc[3]) const {
    impl_->refreshHighRateFeedback();
    if (!acc || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_imu_acc.begin(), impl_->observed_imu_acc.end(), acc);
    return true;
}

bool JointLevelControl::getImuOmegaHighRate(float omega[3]) const {
    impl_->refreshHighRateFeedback();
    if (!omega || !impl_->high_rate_joint_seq || !impl_->high_rate_joint_timestamp) return false;
    std::copy(impl_->observed_imu_omega.begin(), impl_->observed_imu_omega.end(), omega);
    return true;
}

bool JointLevelControl::getJointStateTimestampHighRate(float* time_stamp) const {
    impl_->refreshHighRateFeedback();
    if (!time_stamp || !impl_->high_rate_joint_timestamp) return false;
    *time_stamp = *impl_->high_rate_joint_timestamp;
    return true;
}

bool JointLevelControl::getJointStateSeqHighRate(uint32_t* seq) const {
    impl_->refreshHighRateFeedback();
    if (!seq || !impl_->high_rate_joint_seq) return false;
    *seq = *impl_->high_rate_joint_seq;
    return true;
}

uint8_t JointLevelControl::hostServerMode() const { return 2; }

}  // namespace bpx_sdk

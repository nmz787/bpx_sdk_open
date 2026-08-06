#include "joint_level_control.h"

#include <algorithm>
#include <memory>

namespace bpx_sdk {

class JointLevelControl::Impl {
public:
    std::array<float, 12> kp{};
    std::array<float, 12> pos{};
    std::array<float, 12> kd{};
    std::array<float, 12> vel{};
    std::array<float, 12> tff{};
    float high_rate_joint_timestamp = 0.0f;
    uint32_t high_rate_joint_seq = 0;
};

JointLevelControl::JointLevelControl()
    : impl_(std::make_unique<Impl>()) {}

JointLevelControl::~JointLevelControl() = default;

bool JointLevelControl::connect() { return RequestRobotState::connect(); }
void JointLevelControl::disconnect() { RequestRobotState::disconnect(); }
void JointLevelControl::setJointStateUploadPort(uint16_t port) { RequestRobotState::setJointStateUploadPort(port); }

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
    ++impl_->high_rate_joint_seq;
    return true;
}

bool JointLevelControl::setJointKp(const std::array<float, 12>& kp) { impl_->kp = kp; return true; }
bool JointLevelControl::setJointPosition(const std::array<float, 12>& pos) { impl_->pos = pos; ++impl_->high_rate_joint_seq; return true; }
bool JointLevelControl::setJointKd(const std::array<float, 12>& kd) { impl_->kd = kd; return true; }
bool JointLevelControl::setJointVelocity(const std::array<float, 12>& vel) { impl_->vel = vel; return true; }
bool JointLevelControl::setJointTorqueFeedForward(const std::array<float, 12>& tff) { impl_->tff = tff; return true; }

bool JointLevelControl::setZeroJointCommand() {
    impl_->kp.fill(0.0f);
    impl_->pos.fill(0.0f);
    impl_->kd.fill(0.0f);
    impl_->vel.fill(0.0f);
    impl_->tff.fill(0.0f);
    ++impl_->high_rate_joint_seq;
    return true;
}

bool JointLevelControl::getJointPositionHighRate(float joint_pos[12]) const {
    if (!joint_pos) return false;
    std::copy(impl_->pos.begin(), impl_->pos.end(), joint_pos);
    return true;
}

bool JointLevelControl::getJointVelocityHighRate(float joint_vel[12]) const {
    if (!joint_vel) return false;
    std::copy(impl_->vel.begin(), impl_->vel.end(), joint_vel);
    return true;
}

bool JointLevelControl::getJointTorqueHighRate(float joint_tau[12]) const {
    if (!joint_tau) return false;
    std::copy(impl_->tff.begin(), impl_->tff.end(), joint_tau);
    return true;
}

bool JointLevelControl::getImuRpyHighRate(float rpy[3]) const { return getImuRpy(rpy); }
bool JointLevelControl::getImuQuatHighRate(float quat[4]) const { return getImuQuat(quat); }
bool JointLevelControl::getImuAccHighRate(float acc[3]) const { return getImuAcc(acc); }
bool JointLevelControl::getImuOmegaHighRate(float omega[3]) const { return getImuOmega(omega); }

bool JointLevelControl::getJointStateTimestampHighRate(float* time_stamp) const {
    if (!time_stamp) return false;
    *time_stamp = impl_->high_rate_joint_timestamp;
    return true;
}

bool JointLevelControl::getJointStateSeqHighRate(uint32_t* seq) const {
    if (!seq) return false;
    *seq = impl_->high_rate_joint_seq;
    return true;
}

uint8_t JointLevelControl::hostServerMode() const { return 2; }

}  // namespace bpx_sdk

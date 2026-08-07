#include "motion_level_control.h"

#include "motion_command_sender.h"
#include "recovery_runtime.h"

#include <memory>

namespace bpx_sdk {
namespace {

constexpr int8_t kSubGaitNone = 0;
constexpr int8_t kSubGaitForwardHandstand = -1;
constexpr int8_t kSubGaitBackwardHandstand = 1;
constexpr int8_t kSubGaitLeftFlip = -1;
constexpr int8_t kSubGaitRightFlip = -2;
constexpr int8_t kSubGaitPronk = -1;
constexpr int8_t kSubGaitBound = 1;
constexpr int8_t kSubGaitPace = 2;

}  // namespace

class MotionLevelControl::Impl {
public:
    uint16_t motion_command_rate_hz = 50;
    bool velocity_control_enabled = false;
    bool zero_positions_flag = false;
    MotionGait selected_gait = MotionGait::Walk;
    int8_t selected_sub_gait = kSubGaitNone;
    float velocity_x = 0.0f;
    float velocity_y = 0.0f;
    float velocity_yaw = 0.0f;
    std::unique_ptr<MotionCommandSender> sender;
};

MotionLevelControl::MotionLevelControl()
    : impl_(std::make_unique<Impl>()) {}

MotionLevelControl::~MotionLevelControl() = default;

bool MotionLevelControl::connect() {
    if (!RequestRobotState::connect()) {
        return false;
    }
    if (!impl_->sender) {
        impl_->sender = std::make_unique<MotionCommandSender>();
    }
    if (!impl_->sender->connect(impl_->motion_command_rate_hz)) {
        return false;
    }
    setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
    setCurrentMotionStateValue(MotionState::Passive);
    setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    return true;
}

void MotionLevelControl::disconnect() {
    if (impl_->sender) {
        impl_->sender->disconnect();
    }
    RequestRobotState::disconnect();
}
void MotionLevelControl::setMotionCommandRate(uint16_t rate_hz) { impl_->motion_command_rate_hz = rate_hz; }
void MotionLevelControl::setVelocityControlFlag(bool enabled) {
    impl_->velocity_control_enabled = enabled;
    if (impl_->sender) {
        impl_->sender->setVelocityControlFlag(enabled);
    }
}

void MotionLevelControl::setZeroPositionsFlag() {
    impl_->zero_positions_flag = true;
    if (impl_->sender) {
        impl_->sender->setZeroPositionsFlag();
    }
}

void MotionLevelControl::setWalk() {
    impl_->selected_gait = MotionGait::Walk;
    impl_->selected_sub_gait = kSubGaitNone;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setRunning() {
    impl_->selected_gait = MotionGait::Running;
    impl_->selected_sub_gait = kSubGaitNone;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setLeftFlip() {
    impl_->selected_gait = MotionGait::Flip;
    impl_->selected_sub_gait = kSubGaitLeftFlip;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setRightFlip() {
    impl_->selected_gait = MotionGait::Flip;
    impl_->selected_sub_gait = kSubGaitRightFlip;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setBipedal() {
    impl_->selected_gait = MotionGait::Bipedal;
    impl_->selected_sub_gait = kSubGaitBackwardHandstand;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setInvBipedal() {
    impl_->selected_gait = MotionGait::Bipedal;
    impl_->selected_sub_gait = kSubGaitForwardHandstand;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setPronk() {
    impl_->selected_gait = MotionGait::WalkPhase;
    impl_->selected_sub_gait = kSubGaitPronk;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setPace() {
    impl_->selected_gait = MotionGait::WalkPhase;
    impl_->selected_sub_gait = kSubGaitPace;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

void MotionLevelControl::setBound() {
    impl_->selected_gait = MotionGait::WalkPhase;
    impl_->selected_sub_gait = kSubGaitBound;
    if (impl_->sender) {
        impl_->sender->setGait(static_cast<int>(impl_->selected_gait), static_cast<uint8_t>(impl_->selected_sub_gait));
        impl_->sender->sendLatest();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
}

bool MotionLevelControl::setVelocity(float x, float y, float yaw) {
    impl_->velocity_x = x;
    impl_->velocity_y = y;
    impl_->velocity_yaw = yaw;
    if (impl_->sender) {
        impl_->sender->sendVelocity(x, y, yaw);
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Motion);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
    return true;
}

bool MotionLevelControl::setStandUp() {
    if (impl_->sender) {
        impl_->sender->sendStandUp();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::StandingUp);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
    return true;
}

bool MotionLevelControl::setSitDown() {
    if (impl_->sender) {
        impl_->sender->sendSitDown();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::SitDown);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
    return true;
}

bool MotionLevelControl::setDamping() {
    if (impl_->sender) {
        impl_->sender->sendDamping();
    }
    if (isConnected()) {
        setCurrentGaitState(impl_->selected_gait, impl_->selected_sub_gait);
        setCurrentMotionStateValue(MotionState::Passive);
        setMaxVelocityState(gaitVelocityLimit(impl_->selected_gait));
    }
    return true;
}

uint8_t MotionLevelControl::hostServerMode() const { return 1; }

}  // namespace bpx_sdk

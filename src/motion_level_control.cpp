#include "motion_level_control.h"

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
};

MotionLevelControl::MotionLevelControl()
    : impl_(std::make_unique<Impl>()) {}

MotionLevelControl::~MotionLevelControl() = default;

bool MotionLevelControl::connect() { return RequestRobotState::connect(); }
void MotionLevelControl::disconnect() { RequestRobotState::disconnect(); }
void MotionLevelControl::setMotionCommandRate(uint16_t rate_hz) { impl_->motion_command_rate_hz = rate_hz; }
void MotionLevelControl::setVelocityControlFlag(bool enabled) { impl_->velocity_control_enabled = enabled; }
void MotionLevelControl::setZeroPositionsFlag() { impl_->zero_positions_flag = true; }
void MotionLevelControl::setWalk() { impl_->selected_gait = MotionGait::Walk; impl_->selected_sub_gait = kSubGaitNone; }
void MotionLevelControl::setRunning() { impl_->selected_gait = MotionGait::Running; impl_->selected_sub_gait = kSubGaitNone; }
void MotionLevelControl::setLeftFlip() { impl_->selected_gait = MotionGait::Flip; impl_->selected_sub_gait = kSubGaitLeftFlip; }
void MotionLevelControl::setRightFlip() { impl_->selected_gait = MotionGait::Flip; impl_->selected_sub_gait = kSubGaitRightFlip; }
void MotionLevelControl::setBipedal() { impl_->selected_gait = MotionGait::Bipedal; impl_->selected_sub_gait = kSubGaitBackwardHandstand; }
void MotionLevelControl::setInvBipedal() { impl_->selected_gait = MotionGait::Bipedal; impl_->selected_sub_gait = kSubGaitForwardHandstand; }
void MotionLevelControl::setPronk() { impl_->selected_gait = MotionGait::WalkPhase; impl_->selected_sub_gait = kSubGaitPronk; }
void MotionLevelControl::setPace() { impl_->selected_gait = MotionGait::WalkPhase; impl_->selected_sub_gait = kSubGaitPace; }
void MotionLevelControl::setBound() { impl_->selected_gait = MotionGait::WalkPhase; impl_->selected_sub_gait = kSubGaitBound; }

bool MotionLevelControl::setVelocity(float x, float y, float yaw) {
    impl_->velocity_x = x;
    impl_->velocity_y = y;
    impl_->velocity_yaw = yaw;
    return true;
}

bool MotionLevelControl::setStandUp() { return true; }
bool MotionLevelControl::setSitDown() { return true; }
bool MotionLevelControl::setDamping() { return true; }
uint8_t MotionLevelControl::hostServerMode() const { return 1; }

}  // namespace bpx_sdk

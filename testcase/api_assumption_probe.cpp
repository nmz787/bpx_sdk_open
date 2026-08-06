#include "joint_level_control.h"
#include "motion_level_control.h"
#include "request_robot_state.h"

#include <array>
#include <cstdint>
#include <iostream>

namespace {

class RequestRobotStateProbe : public bpx_sdk::RequestRobotState {
public:
    using bpx_sdk::RequestRobotState::hostServerMode;
    using bpx_sdk::RequestRobotState::robotIp;
};

class MotionLevelControlProbe : public bpx_sdk::MotionLevelControl {
};

class JointLevelControlProbe : public bpx_sdk::JointLevelControl {
};

void printBool(const char* key, bool value) {
    std::cout << key << '=' << (value ? "true" : "false") << '\n';
}

template <typename T>
void printInt(const char* key, T value) {
    std::cout << key << '=' << static_cast<long long>(value) << '\n';
}

}  // namespace

int main() {
    RequestRobotStateProbe state;
    MotionLevelControlProbe motion;
    JointLevelControlProbe joint;

    float joint_array[12] = {};
    float imu_rpy[3] = {};
    float imu_quat[4] = {};
    float scalar = 0.0f;
    std::uint8_t byte = 0;
    std::uint32_t u32 = 0;
    bpx_sdk::LegOdom odom;
    bpx_sdk::MotionState motion_state = bpx_sdk::MotionState::Passive;
    bpx_sdk::MotionGait motion_gait = bpx_sdk::MotionGait::Walk;
    const std::array<float, 12> zeros{};

    printInt("request.hostServerMode", state.hostServerMode());
    std::cout << "request.defaultRobotIp=" << state.robotIp() << '\n';
    state.setRobotIp("192.168.0.42");
    std::cout << "request.customRobotIp=" << state.robotIp() << '\n';
    state.setRobotIp(nullptr);
    std::cout << "request.resetRobotIp=" << state.robotIp() << '\n';

    printBool("request.getRobotVersion.beforeConnect",
              state.getRobotVersion(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr));

    printBool("request.getJointPosition.null", state.getJointPosition(nullptr));
    printBool("request.getJointPosition.fresh", state.getJointPosition(joint_array));
    printBool("request.getJointPositionArray.fresh", state.getJointPositionArray().has_value());
    printBool("request.getImuRpy.fresh", state.getImuRpy(imu_rpy));
    printBool("request.getImuRpyArray.fresh", state.getImuRpyArray().has_value());
    printBool("request.getImuQuat.fresh", state.getImuQuat(imu_quat));
    printBool("request.getImuQuatArray.fresh", state.getImuQuatArray().has_value());
    printBool("request.getLegOdom.null", state.getLegOdom(nullptr));
    printBool("request.getLegOdom.fresh", state.getLegOdom(&odom));
    printBool("request.getLegOdomValue.fresh", state.getLegOdomValue().has_value());
    printBool("request.getCurrentMotionState.byte.fresh", state.getCurrentMotionState(&byte));
    printBool("request.getCurrentMotionState.enum.fresh", state.getCurrentMotionState(&motion_state));
    printBool("request.getCurrentMotionStateValue.fresh", state.getCurrentMotionStateValue().has_value());
    printBool("request.getCurrentMotionStateEnum.fresh", state.getCurrentMotionStateEnum().has_value());
    printBool("request.getCurrentGait.byte.fresh", state.getCurrentGait(&byte));
    printBool("request.getCurrentGait.enum.fresh", state.getCurrentGait(&motion_gait));
    printBool("request.getCurrentGaitValue.fresh", state.getCurrentGaitValue().has_value());
    printBool("request.getCurrentGaitEnum.fresh", state.getCurrentGaitEnum().has_value());
    printBool("request.getBatteryLevel.fresh", state.getBatteryLevel(&byte));
    printBool("request.getBatteryLevelValue.fresh", state.getBatteryLevelValue().has_value());
    printBool("request.getBatteryCurrent.fresh", state.getBatteryCurrent(&scalar));
    printBool("request.getBatteryCurrentValue.fresh", state.getBatteryCurrentValue().has_value());
    printBool("request.getJointStateTimestamp.fresh", state.getJointStateTimestamp(&u32));
    printBool("request.getJointStateTimestampValue.fresh", state.getJointStateTimestampValue().has_value());

    motion.setWalk();
    motion.setBound();
    motion.setVelocityControlFlag(true);
    motion.setVelocity(0.1f, 0.2f, 0.3f);
    motion.setStandUp();
    motion.setDamping();
    printBool("motion.feedbackAfterCommands.state", motion.getCurrentMotionState(&byte));
    printBool("motion.feedbackAfterCommands.gait", motion.getCurrentGait(&byte));
    printBool("motion.feedbackAfterCommands.subGait", motion.getSubGait(&byte));

    joint.setJointCommand(zeros, zeros, zeros, zeros, zeros);
    joint.setJointPosition(zeros);
    joint.setJointVelocity(zeros);
    joint.setJointTorqueFeedForward(zeros);
    joint.setZeroJointCommand();
    printBool("joint.highRatePosition.afterCommands", joint.getJointPositionHighRate(joint_array));
    printBool("joint.highRateVelocity.afterCommands", joint.getJointVelocityHighRate(joint_array));
    printBool("joint.highRateTorque.afterCommands", joint.getJointTorqueHighRate(joint_array));
    printBool("joint.highRateTimestamp.afterCommands", joint.getJointStateTimestampHighRate(&scalar));
    printBool("joint.highRateSeq.afterCommands", joint.getJointStateSeqHighRate(&u32));

    return 0;
}

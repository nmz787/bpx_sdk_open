#ifndef BPX_SDK_MOTION_COMMAND_SENDER_H_
#define BPX_SDK_MOTION_COMMAND_SENDER_H_

#include "recovery_runtime.h"

#include <cstdint>

namespace bpx_sdk {

class RobotStateUdpReceiver;

enum class MotionCommand {
    None,
    StandUp,
    SitDown,
    Damping,
    Velocity,
};

class MotionCommandSender {
public:
    MotionCommandSender();
    ~MotionCommandSender();

    void disconnect();
    bool sendLatest();
    bool sendPacket(MotionCommand command);
    bool sendDamping();
    bool sendSitDown();
    bool sendStandUp();
    bool sendVelocity(float x, float y, float yaw);
    void setControlLock(bool locked);
    void setSubGaitType(unsigned char sub_gait);
    void setZeroPositionsFlag();
    void setVelocityControlFlag(bool enabled);
    bool open();
    void close();
    bool connect(unsigned short rate_hz);
    void runLoop(unsigned short rate_hz);
    void setGait(int gait, unsigned char sub_gait);

    void attachReceiver(RobotStateUdpReceiver* receiver);
    bool isConnected() const;

private:
    bool connected_ = false;
    bool control_lock_ = false;
    bool velocity_control_enabled_ = false;
    bool zero_positions_flag_ = false;
    MotionGait gait_ = MotionGait::Walk;
    int8_t sub_gait_ = 0;
    float velocity_x_ = 0.0f;
    float velocity_y_ = 0.0f;
    float velocity_yaw_ = 0.0f;
    uint16_t rate_hz_ = 0;
    RobotStateUdpReceiver* receiver_ = nullptr;
};

}  // namespace bpx_sdk

#endif  // BPX_SDK_MOTION_COMMAND_SENDER_H_

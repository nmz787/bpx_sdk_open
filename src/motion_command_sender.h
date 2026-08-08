#ifndef BPX_SDK_MOTION_COMMAND_SENDER_H_
#define BPX_SDK_MOTION_COMMAND_SENDER_H_

#include "recovery_runtime.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

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
    void setRobotIp(const char* ip);

    void attachReceiver(RobotStateUdpReceiver* receiver);
    bool isConnected() const;

private:
    std::string robot_ip_;
    int socket_fd_ = -1;
    std::atomic<bool> connected_{false};
    std::thread loop_thread_;
    mutable std::mutex state_mutex_;
    uint8_t command_ = static_cast<uint8_t>(MotionState::Motion);
    uint8_t gait_ = static_cast<uint8_t>(MotionGait::Walk);
    std::array<float, 6> command_values_{};
    uint8_t velocity_control_enabled_ = 0;
    uint8_t zero_positions_nonce_ = 0;
    uint8_t sub_gait_ = 0;
    uint8_t reserved_ = 0;
    uint32_t control_flags_ = 0;
    std::array<uint8_t, 16> reserved_tail_{};
    std::atomic<int> seq_{0};
    RobotStateUdpReceiver* receiver_ = nullptr;
};

}  // namespace bpx_sdk

#endif  // BPX_SDK_MOTION_COMMAND_SENDER_H_

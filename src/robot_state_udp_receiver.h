#ifndef BPX_SDK_ROBOT_STATE_UDP_RECEIVER_H_
#define BPX_SDK_ROBOT_STATE_UDP_RECEIVER_H_

#include "bpx_sdk_config.h"
#include "recovery_runtime.h"

#include <mutex>

namespace bpx_sdk {

class RobotStateUdpReceiver {
public:
    explicit RobotStateUdpReceiver(uint16_t listen_port = DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
    ~RobotStateUdpReceiver();

    void disconnect();
    bool openSocket();
    void closeSocket();
    bool parsePacket(const unsigned char* data, unsigned long size, bool high_rate);
    void receiveLoop(bool high_rate);
    void setListenPort(uint16_t listen_port);
    void run();
    bool connect();

    bool getLatestState(RobotStateSnapshot* snapshot) const;
    void storeLatest(const RobotStateSnapshot& snapshot);

    bool getImuQuat(float* quat) const;
    bool getLegOdom(LegOdom* leg_odom) const;
    bool getSubGait(unsigned char* sub_gait) const;
    void print200Hz(const ClientUploadPacketHead&, const ClientUploadData200Hz&) const;
    bool getImuOmega(float* omega) const;
    bool getLastGait(unsigned char* last_gait) const;
    void print1000Hz(const ClientUploadPacketHead&, const ClientUploadData1000Hz&) const;
    bool getCurrentGait(unsigned char* current_gait) const;
    std::array<float, 3> getImuAccArray() const;
    std::array<float, 3> getImuRpyArray() const;
    bool getJointTorque(float* joint_tau) const;
    bool getLatestState(RobotStateSnapshot* snapshot);
    bool getMaxVelocity(float* max_velocity) const;
    bool getBatteryLevel(unsigned char* battery_level) const;
    std::array<float, 4> getImuQuatArray() const;
    LegOdom getLegOdomValue() const;
    unsigned char getSubGaitValue() const;
    bool getTimestamp1Hz(unsigned int* timestamp) const;
    std::array<float, 3> getImuOmegaArray() const;
    bool getJointPosition(float* joint_pos) const;
    bool getJointVelocity(float* joint_vel) const;
    unsigned char getLastGaitValue() const;
    bool getTimestamp10Hz(unsigned int* timestamp) const;
    bool getTimestamp50Hz(unsigned int* timestamp) const;
    bool getBatteryCurrent(float* battery_current) const;
    bool getTimestamp200Hz(unsigned int* timestamp) const;
    bool getLastMotionState(unsigned char* last_motion_state) const;
    bool getTimestamp1000Hz(unsigned int* timestamp) const;
    unsigned char getCurrentGaitValue() const;
    std::array<float, 12> getJointTorqueArray() const;
    std::array<float, 3> getMaxVelocityArray() const;
    bool getMotorTemperature(float* motor_temperature) const;
    unsigned char getBatteryLevelValue() const;
    bool getDriverTemperature(float* driver_temperature) const;
    unsigned int getTimestamp1HzValue() const;
    bool getCurrentMotionState(unsigned char* current_motion_state) const;
    std::array<float, 12> getJointPositionArray() const;
    std::array<float, 12> getJointVelocityArray() const;
    unsigned int getTimestamp10HzValue() const;
    unsigned int getTimestamp50HzValue() const;
    float getBatteryCurrentValue() const;
    unsigned int getTimestamp200HzValue() const;
    unsigned char getLastMotionStateValue() const;
    unsigned int getTimestamp1000HzValue() const;
    std::array<float, 12> getMotorTemperatureArray() const;
    std::array<float, 12> getDriverTemperatureArray() const;
    unsigned char getCurrentMotionStateValue() const;
    void print1Hz(const ClientUploadPacketHead&, const ClientUploadData1Hz&) const;
    bool getImuAcc(float* acc) const;
    bool getImuRpy(float* rpy) const;
    void print10Hz(const ClientUploadPacketHead&, const ClientUploadData10Hz&) const;
    void print50Hz(const ClientUploadPacketHead&, const ClientUploadData50Hz&) const;

private:
    bool has_snapshot_ = false;
    bool socket_open_ = false;
    uint16_t listen_port_;
    RobotStateSnapshot latest_snapshot_;
    mutable std::mutex mutex_;
};

}  // namespace bpx_sdk

#endif  // BPX_SDK_ROBOT_STATE_UDP_RECEIVER_H_

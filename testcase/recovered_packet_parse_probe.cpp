#include "../src/recovery_runtime.h"
#include "../src/robot_state_udp_receiver.h"

#include <array>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint16_t kPayloadType1000Hz = 0x1000;
constexpr uint16_t kPayloadType200Hz = 0x0200;
constexpr uint16_t kPayloadType50Hz = 0x0050;
constexpr uint16_t kPayloadType10Hz = 0x0010;
constexpr uint16_t kPayloadType1Hz = 0x0001;

bool closeEnough(float lhs, float rhs) {
    return std::fabs(lhs - rhs) < 1e-5f;
}

int fail(const char* message) {
    std::cerr << message << '\n';
    return 1;
}

template <typename T>
std::vector<unsigned char> makePacket(uint16_t payload_type, uint32_t seq, uint32_t timestamp_ms,
                                      const T& payload) {
    bpx_sdk::ClientUploadPacketHead head;
    head.seq = seq;
    head.timestamp_ms = timestamp_ms;
    head.payload_size = static_cast<uint16_t>(sizeof(T));
    head.payload_type = payload_type;

    std::vector<unsigned char> packet(sizeof(head) + sizeof(T));
    std::memcpy(packet.data(), &head, sizeof(head));
    std::memcpy(packet.data() + sizeof(head), &payload, sizeof(T));
    return packet;
}

}  // namespace

int main() {
    bpx_sdk::RobotStateUdpReceiver receiver;
    if (!receiver.connect()) {
        return fail("receiver connect failed");
    }

    bpx_sdk::ClientUploadData1000Hz joint_payload;
    for (size_t i = 0; i < joint_payload.joint_position.size(); ++i) {
        joint_payload.joint_position[i] = static_cast<float>(i) + 0.25f;
        joint_payload.joint_velocity[i] = static_cast<float>(i) - 1.5f;
        joint_payload.joint_torque[i] = static_cast<float>(i) * 2.0f;
    }
    auto joint_packet = makePacket(kPayloadType1000Hz, 11u, 1001u, joint_payload);
    if (!receiver.parsePacket(joint_packet.data(), joint_packet.size(), false)) {
        return fail("1000Hz packet parse failed");
    }

    bpx_sdk::ClientUploadData200Hz imu_payload;
    imu_payload.imu_rpy = {0.1f, -0.2f, 0.3f};
    imu_payload.imu_quat = {0.4f, 0.5f, 0.6f, 0.7f};
    imu_payload.imu_acc = {1.1f, 1.2f, 1.3f};
    imu_payload.imu_omega = {-1.0f, -2.0f, -3.0f};
    auto imu_packet = makePacket(kPayloadType200Hz, 12u, 2002u, imu_payload);
    if (!receiver.parsePacket(imu_packet.data(), imu_packet.size(), false)) {
        return fail("200Hz packet parse failed");
    }

    bpx_sdk::ClientUploadData50Hz odom_payload;
    odom_payload.leg_odom.position[0] = 2.5f;
    odom_payload.leg_odom.position[1] = -1.5f;
    odom_payload.leg_odom.position[2] = 0.5f;
    odom_payload.leg_odom.orientation[3] = 1.0f;
    odom_payload.leg_odom.velocity_body[0] = 0.8f;
    odom_payload.leg_odom.angular_velocity[2] = -0.4f;
    auto odom_packet = makePacket(kPayloadType50Hz, 13u, 3003u, odom_payload);
    if (!receiver.parsePacket(odom_packet.data(), odom_packet.size(), false)) {
        return fail("50Hz packet parse failed");
    }

    bpx_sdk::ClientUploadData10Hz motion_payload;
    motion_payload.current_motion_state = static_cast<uint8_t>(bpx_sdk::MotionState::Motion);
    motion_payload.current_gait = static_cast<uint8_t>(bpx_sdk::MotionGait::Running);
    motion_payload.last_motion_state = static_cast<uint8_t>(bpx_sdk::MotionState::StandingUp);
    motion_payload.last_gait = static_cast<uint8_t>(bpx_sdk::MotionGait::Walk);
    motion_payload.sub_gait = -2;
    motion_payload.max_velocity = {3.5f, 1.5f, 2.5f};
    auto motion_packet = makePacket(kPayloadType10Hz, 14u, 4004u, motion_payload);
    if (!receiver.parsePacket(motion_packet.data(), motion_packet.size(), false)) {
        return fail("10Hz packet parse failed");
    }

    bpx_sdk::ClientUploadData1Hz battery_payload;
    battery_payload.battery_level = 87;
    battery_payload.battery_current = 12.5f;
    for (size_t i = 0; i < battery_payload.motor_temperature.size(); ++i) {
        battery_payload.motor_temperature[i] = static_cast<int8_t>(i - 3);
        battery_payload.driver_temperature[i] = static_cast<int8_t>(i + 4);
    }
    auto battery_packet = makePacket(kPayloadType1Hz, 15u, 5005u, battery_payload);
    if (!receiver.parsePacket(battery_packet.data(), battery_packet.size(), false)) {
        return fail("1Hz packet parse failed");
    }

    float joint_position[12] = {};
    float joint_velocity[12] = {};
    float joint_torque[12] = {};
    float imu_rpy[3] = {};
    float imu_quat[4] = {};
    float imu_acc[3] = {};
    float imu_omega[3] = {};
    float max_velocity[3] = {};
    float motor_temperature[12] = {};
    float driver_temperature[12] = {};
    unsigned char motion_state = 0;
    unsigned char gait = 0;
    unsigned char last_motion_state = 0;
    unsigned char last_gait = 0;
    unsigned char sub_gait = 0;
    unsigned char battery_level = 0;
    float battery_current = 0.0f;
    unsigned int timestamp = 0;
    bpx_sdk::LegOdom odom;

    if (!receiver.getJointPosition(joint_position) || !closeEnough(joint_position[11], 11.25f)) {
        return fail("joint positions not updated from packet");
    }
    if (!receiver.getJointVelocity(joint_velocity) || !closeEnough(joint_velocity[1], -0.5f)) {
        return fail("joint velocities not updated from packet");
    }
    if (!receiver.getJointTorque(joint_torque) || !closeEnough(joint_torque[4], 8.0f)) {
        return fail("joint torques not updated from packet");
    }
    if (!receiver.getTimestamp1000Hz(&timestamp) || timestamp != 1001u) {
        return fail("1000Hz timestamp mismatch");
    }

    if (!receiver.getImuRpy(imu_rpy) || !closeEnough(imu_rpy[2], 0.3f)) {
        return fail("imu rpy not updated from packet");
    }
    if (!receiver.getImuQuat(imu_quat) || !closeEnough(imu_quat[3], 0.7f)) {
        return fail("imu quat not updated from packet");
    }
    if (!receiver.getImuAcc(imu_acc) || !closeEnough(imu_acc[0], 1.1f)) {
        return fail("imu acc not updated from packet");
    }
    if (!receiver.getImuOmega(imu_omega) || !closeEnough(imu_omega[2], -3.0f)) {
        return fail("imu omega not updated from packet");
    }
    if (!receiver.getTimestamp200Hz(&timestamp) || timestamp != 2002u) {
        return fail("200Hz timestamp mismatch");
    }

    if (!receiver.getLegOdom(&odom) || !closeEnough(odom.position[0], 2.5f) ||
        !closeEnough(odom.angular_velocity[2], -0.4f)) {
        return fail("odometry not updated from packet");
    }
    if (!receiver.getTimestamp50Hz(&timestamp) || timestamp != 3003u) {
        return fail("50Hz timestamp mismatch");
    }

    if (!receiver.getCurrentMotionState(&motion_state) ||
        motion_state != static_cast<unsigned char>(bpx_sdk::MotionState::Motion)) {
        return fail("current motion state mismatch");
    }
    if (!receiver.getCurrentGait(&gait) ||
        gait != static_cast<unsigned char>(bpx_sdk::MotionGait::Running)) {
        return fail("current gait mismatch");
    }
    if (!receiver.getLastMotionState(&last_motion_state) ||
        last_motion_state != static_cast<unsigned char>(bpx_sdk::MotionState::StandingUp)) {
        return fail("last motion state mismatch");
    }
    if (!receiver.getLastGait(&last_gait) ||
        last_gait != static_cast<unsigned char>(bpx_sdk::MotionGait::Walk)) {
        return fail("last gait mismatch");
    }
    if (!receiver.getSubGait(&sub_gait) || static_cast<int8_t>(sub_gait) != -2) {
        return fail("sub gait mismatch");
    }
    if (!receiver.getMaxVelocity(max_velocity) || !closeEnough(max_velocity[1], 1.5f)) {
        return fail("max velocity mismatch");
    }
    if (!receiver.getTimestamp10Hz(&timestamp) || timestamp != 4004u) {
        return fail("10Hz timestamp mismatch");
    }

    if (!receiver.getBatteryLevel(&battery_level) || battery_level != 87) {
        return fail("battery level mismatch");
    }
    if (!receiver.getBatteryCurrent(&battery_current) || !closeEnough(battery_current, 12.5f)) {
        return fail("battery current mismatch");
    }
    if (!receiver.getMotorTemperature(motor_temperature) || !closeEnough(motor_temperature[0], 57.0f) ||
        !closeEnough(motor_temperature[11], 68.0f)) {
        return fail("motor temperatures mismatch");
    }
    if (!receiver.getDriverTemperature(driver_temperature) || !closeEnough(driver_temperature[0], 64.0f) ||
        !closeEnough(driver_temperature[11], 75.0f)) {
        return fail("driver temperatures mismatch");
    }
    if (!receiver.getTimestamp1Hz(&timestamp) || timestamp != 5005u) {
        return fail("1Hz timestamp mismatch");
    }

    bpx_sdk::RobotStateUdpReceiver live_receiver(19873);
    if (!live_receiver.connect()) {
        return fail("live receiver connect failed");
    }

    int udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd < 0) {
        return fail("failed to open loopback udp sender");
    }

    sockaddr_in live_address{};
    std::memset(&live_address, 0, sizeof(live_address));
    live_address.sin_family = AF_INET;
    live_address.sin_port = htons(19873);
    live_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    auto live_packet = makePacket(kPayloadType10Hz, 16u, 6006u, motion_payload);
    if (sendto(udp_fd, live_packet.data(), live_packet.size(), 0,
               reinterpret_cast<const sockaddr*>(&live_address),
               sizeof(live_address)) != static_cast<ssize_t>(live_packet.size())) {
        close(udp_fd);
        return fail("failed to send loopback packet");
    }
    close(udp_fd);

    bool live_updated = false;
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (live_receiver.getTimestamp10Hz(&timestamp) && timestamp == 6006u) {
            live_updated = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    live_receiver.disconnect();
    if (!live_updated) {
        return fail("live receiveLoop did not ingest the loopback packet");
    }

    return 0;
}

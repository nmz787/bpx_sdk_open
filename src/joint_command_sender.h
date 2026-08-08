#ifndef BPX_SDK_JOINT_COMMAND_SENDER_H_
#define BPX_SDK_JOINT_COMMAND_SENDER_H_

#include "recovery_runtime.h"

#include <cstdint>
#include <string>

namespace bpx_sdk {

class JointStateReceiver;

class JointCommandSender {
public:
    JointCommandSender();
    ~JointCommandSender();

    bool open();
    bool send(const JointCommandPacket& packet);
    void close();
    bool sendZero();
    void setRobotIp(const char* ip);

    void attachReceiver(JointStateReceiver* receiver);
    bool isOpen() const;

private:
    std::string robot_ip_;
    int socket_fd_ = -1;
    uint32_t seq_ = 0;
    JointCommandPacket latest_command_;
    JointStateReceiver* receiver_ = nullptr;
};

}  // namespace bpx_sdk

#endif  // BPX_SDK_JOINT_COMMAND_SENDER_H_

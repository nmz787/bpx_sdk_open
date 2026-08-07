#ifndef BPX_SDK_JOINT_STATE_RECEIVER_H_
#define BPX_SDK_JOINT_STATE_RECEIVER_H_

#include "bpx_sdk_config.h"
#include "recovery_runtime.h"

#include <mutex>

namespace bpx_sdk {

class JointStateReceiver {
public:
    explicit JointStateReceiver(uint16_t listen_port = DEFAULT_CLIENT_JOINT_STATE_UDP_PORT);
    ~JointStateReceiver();

    void receiveLoop();
    bool receiveOnce(JointStatePacket* packet) const;
    void storeLatest(const JointStatePacket& packet);
    void setListenPort(unsigned short listen_port);
    void run();
    bool open();
    void stop();
    void close();
    void start();
    bool getLatest(JointStatePacket* packet) const;

private:
    bool running_ = false;
    bool has_packet_ = false;
    uint16_t listen_port_;
    JointStatePacket latest_packet_;
    mutable std::mutex mutex_;
};

}  // namespace bpx_sdk

#endif  // BPX_SDK_JOINT_STATE_RECEIVER_H_

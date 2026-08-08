#ifndef BPX_SDK_TCP_SUBSCRIBE_CLIENT_H_
#define BPX_SDK_TCP_SUBSCRIBE_CLIENT_H_

#include "recovery_runtime.h"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace bpx_sdk {

class RobotStateUdpReceiver;

class TcpSubscribeClient {
public:
    TcpSubscribeClient();
    ~TcpSubscribeClient();

    void disconnect();
    void setSessionId(uint16_t session_id);
    void setTcpLocalPort(uint16_t local_port);
    bool startStateQuery();
    bool queryRobotVersion(RobotVersionInfo* info);
    void setHostServerMode(unsigned char host_server_mode);
    void setJointStateUploadPort(uint16_t upload_port);
    void setRobotStateUploadPort(uint16_t upload_port);
    void setRobotStateUploadRate(uint16_t upload_rate_hz);
    void setRobotIp(const char* ip);
    bool sendRequest(const SubscribeStateReq& request) const;
    void printResponse(const SubscribeStateResp& response) const;
    bool bindLocalTcpPort(int port) const;
    bool sendDefaultRequest() const;
    bool sendStateQueryRequest() const;
    uint32_t nowMs() const;
    bool recvAll(int socket_fd, unsigned char* buffer, unsigned long size) const;
    bool sendAll(int socket_fd, const unsigned char* buffer, unsigned long size) const;

    void attachReceiver(RobotStateUdpReceiver* receiver);

private:
    std::string robot_ip_;
    uint16_t server_port_ = 0;
    uint16_t session_id_ = 0;
    uint16_t tcp_local_port_ = 0;
    uint16_t joint_state_upload_port_ = 0;
    uint16_t robot_state_upload_port_ = 0;
    uint16_t robot_state_upload_rate_hz_ = 0;
    unsigned char host_server_mode_ = 0;
    mutable int response_socket_fd_ = -1;
    mutable std::atomic<bool> response_loop_running_{false};
    mutable std::mutex response_mutex_;
    mutable std::thread response_thread_;
    RobotStateUdpReceiver* receiver_ = nullptr;
};

}  // namespace bpx_sdk

#endif  // BPX_SDK_TCP_SUBSCRIBE_CLIENT_H_

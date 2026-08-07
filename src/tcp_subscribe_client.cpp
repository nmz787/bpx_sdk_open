#include "tcp_subscribe_client.h"

#include "robot_state_udp_receiver.h"

namespace bpx_sdk {

TcpSubscribeClient::TcpSubscribeClient() = default;
TcpSubscribeClient::~TcpSubscribeClient() = default;

void TcpSubscribeClient::disconnect() {}

void TcpSubscribeClient::setSessionId(uint16_t session_id) {
    session_id_ = session_id;
}

void TcpSubscribeClient::setTcpLocalPort(uint16_t local_port) {
    tcp_local_port_ = local_port;
}

bool TcpSubscribeClient::startStateQuery() {
    SubscribeStateReq request;
    request.session_id = session_id_;
    request.robot_state_upload_port = robot_state_upload_port_;
    request.joint_state_upload_port = joint_state_upload_port_;
    request.robot_state_upload_rate_hz = robot_state_upload_rate_hz_;
    request.host_server_mode = host_server_mode_;
    if (!sendRequest(request)) {
        return false;
    }
    if (receiver_) {
        receiver_->storeLatest(makeConnectedSnapshot());
    }
    return true;
}

bool TcpSubscribeClient::queryRobotVersion(RobotVersionInfo* info) {
    if (!info) {
        return false;
    }
    *info = currentSdkVersion();
    return true;
}

void TcpSubscribeClient::setHostServerMode(unsigned char host_server_mode) {
    host_server_mode_ = host_server_mode;
}

void TcpSubscribeClient::setJointStateUploadPort(uint16_t upload_port) {
    joint_state_upload_port_ = upload_port;
}

void TcpSubscribeClient::setRobotStateUploadPort(uint16_t upload_port) {
    robot_state_upload_port_ = upload_port;
}

void TcpSubscribeClient::setRobotStateUploadRate(uint16_t upload_rate_hz) {
    robot_state_upload_rate_hz_ = upload_rate_hz;
}

bool TcpSubscribeClient::sendRequest(const SubscribeStateReq&) const {
    return true;
}

void TcpSubscribeClient::printResponse(const SubscribeStateResp&) const {}

bool TcpSubscribeClient::bindLocalTcpPort(int) const {
    return true;
}

bool TcpSubscribeClient::sendDefaultRequest() const {
    return true;
}

bool TcpSubscribeClient::sendStateQueryRequest() const {
    return true;
}

uint32_t TcpSubscribeClient::nowMs() const {
    return bpx_sdk::nowMs();
}

bool TcpSubscribeClient::recvAll(int, unsigned char*, unsigned long) const {
    return true;
}

bool TcpSubscribeClient::sendAll(int, const unsigned char*, unsigned long) const {
    return true;
}

void TcpSubscribeClient::attachReceiver(RobotStateUdpReceiver* receiver) {
    receiver_ = receiver;
}

}  // namespace bpx_sdk

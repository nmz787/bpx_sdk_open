# Symbol Recovery Inventory

Recovered from `/home/runner/work/bpx_sdk_open/bpx_sdk_open/lib/libbpx_sdk_x86_64.so`.

This inventory focuses on demangled `bpx_sdk::*` symbols preserved in the unstripped shared library.

## Public API: RequestRobotState

- `bpx_sdk::RequestRobotState::disconnect()`
- `bpx_sdk::RequestRobotState::setRobotIp(char const*)`
- `bpx_sdk::RequestRobotState::setSessionId(unsigned short)`
- `bpx_sdk::RequestRobotState::setTcpLocalPort(unsigned short)`
- `bpx_sdk::RequestRobotState::queryRobotVersion(unsigned short*, unsigned short*, unsigned short*, unsigned int*, unsigned int*, unsigned int*)`
- `bpx_sdk::RequestRobotState::setJointStateUploadPort(unsigned short)`
- `bpx_sdk::RequestRobotState::setRobotStateUploadPort(unsigned short)`
- `bpx_sdk::RequestRobotState::setRobotStateUploadRate(unsigned short)`
- `bpx_sdk::RequestRobotState::connect()`
- `bpx_sdk::RequestRobotState::RequestRobotState()`
- `bpx_sdk::RequestRobotState::~RequestRobotState()`
- `bpx_sdk::RequestRobotState::getImuQuat(float*) const`
- `bpx_sdk::RequestRobotState::getLegOdom(bpx_sdk::LegOdom*) const`
- `bpx_sdk::RequestRobotState::getSubGait(unsigned char*) const`
- `bpx_sdk::RequestRobotState::getImuOmega(float*) const`
- `bpx_sdk::RequestRobotState::getLastGait(bpx_sdk::MotionGait*) const`
- `bpx_sdk::RequestRobotState::getLastGait(unsigned char*) const`
- `bpx_sdk::RequestRobotState::getCurrentGait(bpx_sdk::MotionGait*) const`
- `bpx_sdk::RequestRobotState::getCurrentGait(unsigned char*) const`
- `bpx_sdk::RequestRobotState::getImuAccArray() const`
- `bpx_sdk::RequestRobotState::getImuRpyArray() const`
- `bpx_sdk::RequestRobotState::getJointTorque(float*) const`
- `bpx_sdk::RequestRobotState::getMaxVelocity(float*) const`
- `bpx_sdk::RequestRobotState::hostServerMode() const`
- `bpx_sdk::RequestRobotState::getBatteryLevel(unsigned char*) const`
- `bpx_sdk::RequestRobotState::getImuQuatArray() const`
- `bpx_sdk::RequestRobotState::getImuTimestamp(unsigned int*) const`
- `bpx_sdk::RequestRobotState::getLastGaitEnum() const`
- `bpx_sdk::RequestRobotState::getLegOdomValue() const`
- `bpx_sdk::RequestRobotState::getRobotVersion(unsigned short*, unsigned short*, unsigned short*, unsigned int*, unsigned int*, unsigned int*) const`
- `bpx_sdk::RequestRobotState::getSubGaitValue() const`
- `bpx_sdk::RequestRobotState::getImuOmegaArray() const`
- `bpx_sdk::RequestRobotState::getJointPosition(float*) const`
- `bpx_sdk::RequestRobotState::getJointVelocity(float*) const`
- `bpx_sdk::RequestRobotState::getLastGaitValue() const`
- `bpx_sdk::RequestRobotState::getBatteryCurrent(float*) const`
- `bpx_sdk::RequestRobotState::getCurrentGaitEnum() const`
- `bpx_sdk::RequestRobotState::getLastMotionState(bpx_sdk::MotionState*) const`
- `bpx_sdk::RequestRobotState::getLastMotionState(unsigned char*) const`
- `bpx_sdk::RequestRobotState::getBatteryTimestamp(unsigned int*) const`
- `bpx_sdk::RequestRobotState::getCurrentGaitValue() const`
- `bpx_sdk::RequestRobotState::getJointTorqueArray() const`
- `bpx_sdk::RequestRobotState::getMaxVelocityArray() const`
- `bpx_sdk::RequestRobotState::getMotorTemperature(float*) const`
- `bpx_sdk::RequestRobotState::getBatteryLevelValue() const`
- `bpx_sdk::RequestRobotState::getDriverTemperature(float*) const`
- `bpx_sdk::RequestRobotState::getImuTimestampValue() const`
- `bpx_sdk::RequestRobotState::getOdometryTimestamp(unsigned int*) const`
- `bpx_sdk::RequestRobotState::getCurrentMotionState(bpx_sdk::MotionState*) const`
- `bpx_sdk::RequestRobotState::getCurrentMotionState(unsigned char*) const`
- `bpx_sdk::RequestRobotState::getJointPositionArray() const`
- `bpx_sdk::RequestRobotState::getJointVelocityArray() const`
- `bpx_sdk::RequestRobotState::getBatteryCurrentValue() const`
- `bpx_sdk::RequestRobotState::getJointStateTimestamp(unsigned int*) const`
- `bpx_sdk::RequestRobotState::getLastMotionStateEnum() const`
- `bpx_sdk::RequestRobotState::getLastMotionStateValue() const`
- `bpx_sdk::RequestRobotState::getMotionStateTimestamp(unsigned int*) const`
- `bpx_sdk::RequestRobotState::getBatteryTimestampValue() const`
- `bpx_sdk::RequestRobotState::getMotorTemperatureArray() const`
- `bpx_sdk::RequestRobotState::getCurrentMotionStateEnum() const`
- `bpx_sdk::RequestRobotState::getDriverTemperatureArray() const`
- `bpx_sdk::RequestRobotState::getOdometryTimestampValue() const`
- `bpx_sdk::RequestRobotState::getCurrentMotionStateValue() const`
- `bpx_sdk::RequestRobotState::getJointStateTimestampValue() const`
- `bpx_sdk::RequestRobotState::getMotionStateTimestampValue() const`
- `bpx_sdk::RequestRobotState::robotIp() const`
- `bpx_sdk::RequestRobotState::getImuAcc(float*) const`
- `bpx_sdk::RequestRobotState::getImuRpy(float*) const`

## Public API: MotionLevelControl

- `bpx_sdk::MotionLevelControl::disconnect()`
- `bpx_sdk::MotionLevelControl::setBipedal()`
- `bpx_sdk::MotionLevelControl::setDamping()`
- `bpx_sdk::MotionLevelControl::setRunning()`
- `bpx_sdk::MotionLevelControl::setSitDown()`
- `bpx_sdk::MotionLevelControl::setStandUp()`
- `bpx_sdk::MotionLevelControl::setLeftFlip()`
- `bpx_sdk::MotionLevelControl::setVelocity(float, float, float)`
- `bpx_sdk::MotionLevelControl::setRightFlip()`
- `bpx_sdk::MotionLevelControl::setInvBipedal()`
- `bpx_sdk::MotionLevelControl::setMotionCommandRate(unsigned short)`
- `bpx_sdk::MotionLevelControl::setZeroPositionsFlag()`
- `bpx_sdk::MotionLevelControl::setVelocityControlFlag(bool)`
- `bpx_sdk::MotionLevelControl::connect()`
- `bpx_sdk::MotionLevelControl::setPace()`
- `bpx_sdk::MotionLevelControl::setWalk()`
- `bpx_sdk::MotionLevelControl::setBound()`
- `bpx_sdk::MotionLevelControl::setPronk()`
- `bpx_sdk::MotionLevelControl::MotionLevelControl()`
- `bpx_sdk::MotionLevelControl::~MotionLevelControl()`
- `bpx_sdk::MotionLevelControl::hostServerMode() const`

## Public API: JointLevelControl

- `bpx_sdk::JointLevelControl::disconnect()`
- `bpx_sdk::JointLevelControl::setZeroJointCommand()`
- `bpx_sdk::JointLevelControl::setJointStateUploadPort(unsigned short)`
- `bpx_sdk::JointLevelControl::setJointCommand(std::array<float, 12ul> const&, std::array<float, 12ul> const&, std::array<float, 12ul> const&, std::array<float, 12ul> const&, std::array<float, 12ul> const&)`
- `bpx_sdk::JointLevelControl::setJointKp(std::array<float, 12ul> const&)`
- `bpx_sdk::JointLevelControl::setJointPosition(std::array<float, 12ul> const&)`
- `bpx_sdk::JointLevelControl::setJointKd(std::array<float, 12ul> const&)`
- `bpx_sdk::JointLevelControl::setJointVelocity(std::array<float, 12ul> const&)`
- `bpx_sdk::JointLevelControl::setJointTorqueFeedForward(std::array<float, 12ul> const&)`
- `bpx_sdk::JointLevelControl::connect()`
- `bpx_sdk::JointLevelControl::JointLevelControl()`
- `bpx_sdk::JointLevelControl::~JointLevelControl()`
- `bpx_sdk::JointLevelControl::hostServerMode() const`
- `bpx_sdk::JointLevelControl::getImuAccHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getImuRpyHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getImuQuatHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getImuOmegaHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getJointTorqueHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getJointPositionHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getJointStateSeqHighRate(unsigned int*) const`
- `bpx_sdk::JointLevelControl::getJointVelocityHighRate(float*) const`
- `bpx_sdk::JointLevelControl::getJointStateTimestampHighRate(float*) const`

## Internal Class: JointCommandSender

- `bpx_sdk::JointCommandSender::open()`
- `bpx_sdk::JointCommandSender::send(bpx_sdk::JointCommandPacket const&)`
- `bpx_sdk::JointCommandSender::close()`
- `bpx_sdk::JointCommandSender::sendZero()`
- `bpx_sdk::JointCommandSender::~JointCommandSender()`

## Internal Class: JointStateReceiver

- `bpx_sdk::JointStateReceiver::receiveLoop()`
- `bpx_sdk::JointStateReceiver::receiveOnce(bpx_sdk::JointStatePacket*)`
- `bpx_sdk::JointStateReceiver::storeLatest(bpx_sdk::JointStatePacket const&)`
- `bpx_sdk::JointStateReceiver::setListenPort(unsigned short)`
- `bpx_sdk::JointStateReceiver::run()`
- `bpx_sdk::JointStateReceiver::open()`
- `bpx_sdk::JointStateReceiver::stop()`
- `bpx_sdk::JointStateReceiver::close()`
- `bpx_sdk::JointStateReceiver::start()`
- `bpx_sdk::JointStateReceiver::JointStateReceiver(unsigned short)`
- `bpx_sdk::JointStateReceiver::~JointStateReceiver()`
- `bpx_sdk::JointStateReceiver::getLatest(bpx_sdk::JointStatePacket*) const`

## Internal Class: MotionCommandSender

- `bpx_sdk::MotionCommandSender::disconnect()`
- `bpx_sdk::MotionCommandSender::sendLatest()`
- `bpx_sdk::MotionCommandSender::sendPacket(bpx_sdk::MotionCommand)`
- `bpx_sdk::MotionCommandSender::sendDamping()`
- `bpx_sdk::MotionCommandSender::sendSitDown()`
- `bpx_sdk::MotionCommandSender::sendStandUp()`
- `bpx_sdk::MotionCommandSender::sendVelocity(float, float, float)`
- `bpx_sdk::MotionCommandSender::setControlLock(bool)`
- `bpx_sdk::MotionCommandSender::setSubGaitType(unsigned char)`
- `bpx_sdk::MotionCommandSender::setZeroPositionsFlag()`
- `bpx_sdk::MotionCommandSender::setVelocityControlFlag(bool)`
- `bpx_sdk::MotionCommandSender::open()`
- `bpx_sdk::MotionCommandSender::close()`
- `bpx_sdk::MotionCommandSender::connect(unsigned short)`
- `bpx_sdk::MotionCommandSender::runLoop(unsigned short)`
- `bpx_sdk::MotionCommandSender::setGait(int, unsigned char)`
- `bpx_sdk::MotionCommandSender::~MotionCommandSender()`

## Internal Class: TcpSubscribeClient

- `bpx_sdk::TcpSubscribeClient::disconnect()`
- `bpx_sdk::TcpSubscribeClient::setSessionId(unsigned short)`
- `bpx_sdk::TcpSubscribeClient::setTcpLocalPort(unsigned short)`
- `bpx_sdk::TcpSubscribeClient::startStateQuery()`
- `bpx_sdk::TcpSubscribeClient::queryRobotVersion(bpx_sdk::RobotVersionInfo*)`
- `bpx_sdk::TcpSubscribeClient::setHostServerMode(unsigned char)`
- `bpx_sdk::TcpSubscribeClient::setJointStateUploadPort(unsigned short)`
- `bpx_sdk::TcpSubscribeClient::setRobotStateUploadPort(unsigned short)`
- `bpx_sdk::TcpSubscribeClient::setRobotStateUploadRate(unsigned short)`
- `bpx_sdk::TcpSubscribeClient::TcpSubscribeClient()`
- `bpx_sdk::TcpSubscribeClient::~TcpSubscribeClient()`
- `bpx_sdk::TcpSubscribeClient::sendRequest(bpx_sdk::SubscribeStateReq const&) const`
- `bpx_sdk::TcpSubscribeClient::printResponse(bpx_sdk::SubscribeStateResp const&) const`
- `bpx_sdk::TcpSubscribeClient::bindLocalTcpPort(int) const`
- `bpx_sdk::TcpSubscribeClient::sendDefaultRequest() const`
- `bpx_sdk::TcpSubscribeClient::sendStateQueryRequest() const`
- `bpx_sdk::TcpSubscribeClient::nowMs() const`
- `bpx_sdk::TcpSubscribeClient::recvAll(int, unsigned char*, unsigned long) const`
- `bpx_sdk::TcpSubscribeClient::sendAll(int, unsigned char const*, unsigned long) const`

## Internal Class: RobotStateUdpReceiver

- `bpx_sdk::RobotStateUdpReceiver::disconnect()`
- `bpx_sdk::RobotStateUdpReceiver::openSocket()`
- `bpx_sdk::RobotStateUdpReceiver::closeSocket()`
- `bpx_sdk::RobotStateUdpReceiver::parsePacket(unsigned char const*, unsigned long, bool)`
- `bpx_sdk::RobotStateUdpReceiver::receiveLoop(bool)`
- `bpx_sdk::RobotStateUdpReceiver::setListenPort(unsigned short)`
- `bpx_sdk::RobotStateUdpReceiver::run()`
- `bpx_sdk::RobotStateUdpReceiver::connect()`
- `bpx_sdk::RobotStateUdpReceiver::RobotStateUdpReceiver(unsigned short)`
- `bpx_sdk::RobotStateUdpReceiver::~RobotStateUdpReceiver()`
- `bpx_sdk::RobotStateUdpReceiver::getImuQuat(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getLegOdom(bpx_sdk::LegOdom*) const`
- `bpx_sdk::RobotStateUdpReceiver::getSubGait(unsigned char*) const`
- `bpx_sdk::RobotStateUdpReceiver::print200Hz(bpx_sdk::ClientUploadPacketHead const&, bpx_sdk::ClientUploadData200Hz const&) const`
- `bpx_sdk::RobotStateUdpReceiver::getImuOmega(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getLastGait(unsigned char*) const`
- `bpx_sdk::RobotStateUdpReceiver::print1000Hz(bpx_sdk::ClientUploadPacketHead const&, bpx_sdk::ClientUploadData1000Hz const&) const`
- `bpx_sdk::RobotStateUdpReceiver::getCurrentGait(unsigned char*) const`
- `bpx_sdk::RobotStateUdpReceiver::getImuAccArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getImuRpyArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getJointTorque(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getLatestState(bpx_sdk::RobotStateSnapshot*) const`
- `bpx_sdk::RobotStateUdpReceiver::getMaxVelocity(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getBatteryLevel(unsigned char*) const`
- `bpx_sdk::RobotStateUdpReceiver::getImuQuatArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getLegOdomValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getSubGaitValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp1Hz(unsigned int*) const`
- `bpx_sdk::RobotStateUdpReceiver::getImuOmegaArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getJointPosition(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getJointVelocity(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getLastGaitValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp10Hz(unsigned int*) const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp50Hz(unsigned int*) const`
- `bpx_sdk::RobotStateUdpReceiver::getBatteryCurrent(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp200Hz(unsigned int*) const`
- `bpx_sdk::RobotStateUdpReceiver::getLastMotionState(unsigned char*) const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp1000Hz(unsigned int*) const`
- `bpx_sdk::RobotStateUdpReceiver::getCurrentGaitValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getJointTorqueArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getMaxVelocityArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getMotorTemperature(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getBatteryLevelValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getDriverTemperature(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp1HzValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getCurrentMotionState(unsigned char*) const`
- `bpx_sdk::RobotStateUdpReceiver::getJointPositionArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getJointVelocityArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp10HzValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp50HzValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getBatteryCurrentValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp200HzValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getLastMotionStateValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getTimestamp1000HzValue() const`
- `bpx_sdk::RobotStateUdpReceiver::getMotorTemperatureArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getDriverTemperatureArray() const`
- `bpx_sdk::RobotStateUdpReceiver::getCurrentMotionStateValue() const`
- `bpx_sdk::RobotStateUdpReceiver::print1Hz(bpx_sdk::ClientUploadPacketHead const&, bpx_sdk::ClientUploadData1Hz const&) const`
- `bpx_sdk::RobotStateUdpReceiver::getImuAcc(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::getImuRpy(float*) const`
- `bpx_sdk::RobotStateUdpReceiver::print10Hz(bpx_sdk::ClientUploadPacketHead const&, bpx_sdk::ClientUploadData10Hz const&) const`
- `bpx_sdk::RobotStateUdpReceiver::print50Hz(bpx_sdk::ClientUploadPacketHead const&, bpx_sdk::ClientUploadData50Hz const&) const`

## Internal Helpers

- `bpx_sdk::socket_compat::closeSocket(int)`
- `bpx_sdk::socket_compat::isWouldBlock(int)`
- `bpx_sdk::socket_compat::setNoSigPipe(int)`
- `bpx_sdk::socket_compat::setReuseAddr(int)`
- `bpx_sdk::socket_compat::shutdownBoth(int)`
- `bpx_sdk::socket_compat::isInterrupted(int)`
- `bpx_sdk::socket_compat::lastErrorCode()`
- `bpx_sdk::socket_compat::setNonBlocking(int)`
- `bpx_sdk::socket_compat::lastErrorMessage[abi:cxx11]()`
- `bpx_sdk::socket_compat::ensureInitialized()`
- `bpx_sdk::socket_compat::setReceiveTimeout(int, int)`
- `bpx_sdk::socket_compat::recv(int, unsigned char*, unsigned long, int)`
- `bpx_sdk::socket_compat::send(int, unsigned char const*, unsigned long, int)`
- `bpx_sdk::socket_compat::sendTo(int, void const*, unsigned long, int, sockaddr const*, unsigned int)`
- `bpx_sdk::socket_compat::recvFrom(int, void*, unsigned long, int, sockaddr*, unsigned int*)`
- `bpx_sdk::EncodeVersion(unsigned short, unsigned short, unsigned short)`
- `bpx_sdk::EncodeCommitHash(char const*)`
- `bpx_sdk::SubscribeStateReq::SubscribeStateReq()`

## Binary-only or Less-obvious Findings

- The shared library preserves extensive internal class names, not just the public API.
- `RobotStateUdpReceiver` exposes multi-rate timestamp getters: 1Hz, 10Hz, 50Hz, 200Hz, and 1000Hz.
- `RobotStateUdpReceiver` also contains debug/inspection helpers such as `print1Hz`, `print10Hz`, `print50Hz`, `print200Hz`, and `print1000Hz`.
- `TcpSubscribeClient` contains explicit request/response helpers like `sendDefaultRequest`, `sendStateQueryRequest`, `sendRequest`, `recvAll`, and `sendAll`.
- `MotionCommandSender` contains gait/control helpers such as `setGait`, `setSubGaitType`, `setControlLock`, `sendLatest`, and `runLoop`.
- The public headers appear to cover the exposed public API well, but the binary reveals a richer internal transport and packet-processing layer.

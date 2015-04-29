#ifndef EVIL_NETWORKCHANNEL_HPP
#define EVIL_NETWORKCHANNEL_HPP

#include <cstdint>
#include <memory>
#include <string>
#include "boost/asio/io_service.hpp"
#include "evil/HardwareChannel.hpp"
#include "evil/RpcInterface.hpp"
#include "evil/ZmqSocket.hpp"
#include "msgpack.hpp"

namespace evil {

class NetworkChannel : public std::enable_shared_from_this<NetworkChannel> {
public:
    static std::shared_ptr<NetworkChannel>
    make(boost::asio::io_service &ioService,
         std::shared_ptr<HardwareChannel> hw) {
        return std::shared_ptr<NetworkChannel>(
            new NetworkChannel(ioService, std::move(hw)));
    }

    void start();

    void stop();

    uint16_t rpcPort() const { return rpcInterface_->port(); }

private:
    NetworkChannel(boost::asio::io_service &ioService,
                   std::shared_ptr<HardwareChannel> hw);

    bool processRpcCommand(const std::string &method, msgpack::object &param);

    template <typename... T>
    void sendNotification(const std::string &name, T &&... values);

    void sendStreamPacket(StreamIdx idx, const StreamPacket &packet);

    void nextMonitorEvent(StreamIdx idx, azmq::socket &socket);

    void addStreamSubscription(StreamIdx idx);
    void removeStreamSubscription(StreamIdx idx);

    boost::asio::io_service &ioService_;
    std::shared_ptr<HardwareChannel> hw_;

    std::shared_ptr<RpcInterface> rpcInterface_;
    ZmqSocket notificationSocket_;
    std::vector<std::unique_ptr<ZmqSocket>> streamingSockets_;
    std::vector<std::unique_ptr<azmq::socket>> streamingMonitorSockets_;
    std::vector<size_t> streamingSubscriberCounts_;

    /// Buffer for sending notifications. Only ever used while sendNotification
    /// is executing, but kept around here so we can reuse the allocation.
    msgpack::sbuffer notificationBuf_;

    /// Same as notificationBuf_, but for streaming packets.
    msgpack::sbuffer streamingBuf_;
};
}

#endif

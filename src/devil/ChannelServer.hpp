#ifndef DEVIL_CHANNELSERVER_HPP
#define DEVIL_CHANNELSERVER_HPP

#include <cstdint>
#include <memory>
#include <string>
#include "boost/asio/io_service.hpp"
#include "boost/asio/steady_timer.hpp"
#include "devil/Channel.hpp"
#include "devil/RpcInterface.hpp"
#include "devil/ZmqSocket.hpp"
#include "msgpack.hpp"

namespace devil {

class ChannelServer : public std::enable_shared_from_this<ChannelServer> {
public:
    static std::shared_ptr<ChannelServer>
    make(boost::asio::io_service &ioService, std::shared_ptr<Channel> hw) {
        return std::shared_ptr<ChannelServer>(
            new ChannelServer(ioService, std::move(hw)));
    }

    void start();

    void stop();

    uint16_t rpcPort() const { return rpcInterface_->port(); }

private:
    ChannelServer(boost::asio::io_service &ioService,
                  std::shared_ptr<Channel> hw);

    bool processRpcCommand(const std::string &method, msgpack::object &param);

    template <typename... T>
    void sendNotification(const std::string &name, T &&... values);

    void sendStreamPacket(StreamIdx idx, const StreamPacket &packet);

    void nextStreamingMonitorTimer();
    void processMonitorEvents(StreamIdx idx, azmq::socket &socket);

    void addStreamSubscription(StreamIdx idx);
    void removeStreamSubscription(StreamIdx idx);

    boost::asio::io_service &ioService_;
    std::shared_ptr<Channel> hw_;

    std::shared_ptr<RpcInterface> rpcInterface_;
    ZmqSocket notificationSocket_;
    std::vector<std::unique_ptr<ZmqSocket>> streamingSockets_;
    std::vector<std::unique_ptr<azmq::socket>> streamingMonitorSockets_;
    boost::asio::steady_timer streamingMonitorTimer_;
    std::vector<size_t> streamingSubscriberCounts_;

    /// Buffer for sending notifications. Only ever used while sendNotification
    /// is executing, but kept around here so we can reuse the allocation.
    msgpack::sbuffer notificationBuf_;

    /// Same as notificationBuf_, but for streaming packets.
    msgpack::sbuffer streamingBuf_;
};
}

#endif

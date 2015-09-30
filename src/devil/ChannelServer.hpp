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

/// \brief Exposes a single devil::Channel via a ZeroMQ/msgpack-rpc interface.
///
/// The interface consists of a single REP socket for RPC requests, a PUB
/// socket for general notifications such as register changes, and one more
/// PUB socket per streaming channel that broadcasts the packets that they are
/// received.
///
/// The implementation keeps track of the number of subscribers on each
/// streaming socket such that the hardware does not need to be queried if the
/// packets are not observed anyway, thus increasing the available bandwidth per
/// device.
class ChannelServer : public std::enable_shared_from_this<ChannelServer> {
public:
    /// \brief Constructs a new instance.
    ///
    /// The constructor itself is not public as the chosen strategy to deal with
    /// memory management in the face of the asynchronous callbacks relies on
    /// the lifetime being managed by std::shared_ptr.
    static std::shared_ptr<ChannelServer>
    make(boost::asio::io_service &ioService, std::shared_ptr<Channel> hw) {
        return std::shared_ptr<ChannelServer>(
            new ChannelServer(ioService, std::move(hw)));
    }

    /// \brief Starts asynchronously listening for network connections.
    ///
    /// Invalid to call more than once, even after #stop() has been invoked.
    void start();

    /// \brief Signals the implementation to exit cleanly at the next possible
    /// point in time.
    ///
    /// Should not leave any active async operations behind so that boost::asio
    /// can be shut down cleanly afterwards.
    void stop();

    /// \brief Returns the TCP port on which the RPC interface will be exposed
    /// (ZeroMQ REP socket).
    ///
    /// Will be assigned by the operating system, and as such no assumptions
    /// should be made about the port chosen.
    uint16_t rpcPort() const { return rpcInterface_->port(); }

private:
    ChannelServer(boost::asio::io_service &ioService,
                  std::shared_ptr<Channel> hw);

    /// Callback invoked by rpcInterface_ to handle new requests on the main
    /// configuration REP socket.
    bool processRpcCommand(const std::string &method, msgpack::object &param);

    /// Sends a msgpack-rpc notification to the main notification PUB socket.
    template <typename... T>
    void sendNotification(const std::string &name, T &&... values);

    /// Sends the given stream packet to subscribers on the associated PUB
    /// socket.
    void sendStreamPacket(StreamIdx idx, const StreamPacket &packet);

    /// Asynchronously waits for the next event on the monitor socket for the
    /// given stream and handles it; continuing asynchronously until the socket
    /// is closed.
    void nextMonitorEvent(StreamIdx idx, azmq::socket &socket);

    /// Keeps track of a new subscriber for the given stream and registers the
    /// hardware callback if necessary.
    void addStreamSubscription(StreamIdx idx);

    /// Removes subscriber for the given stream and unregisters the hardware
    /// callback if it was the last one.
    void removeStreamSubscription(StreamIdx idx);

    boost::asio::io_service &ioService_;

    /// The controller channel exposed to clients.
    std::shared_ptr<Channel> hw_;

    /// Handles the main msgpack-rpc configuration interface.
    std::shared_ptr<RpcInterface> rpcInterface_;

    /// PUB socket for general notification.
    ZmqSocket notificationSocket_;

    /// PUB sockets for each hardware streaming channel. They are active for the
    /// entire operation of the server; the unique_ptr is only needed because
    /// the objects are not movable.
    std::vector<std::unique_ptr<ZmqSocket>> streamingSockets_;

    /// inproc PAIR sockets for listening for ZeroMQ monitor events.
    std::vector<std::unique_ptr<azmq::socket>> streamingMonitorSockets_;

    /// The number of connected subscribers for each streaming channel.
    std::vector<size_t> streamingSubscriberCounts_;

    /// Buffer for sending notifications. Only ever used while sendNotification
    /// is executing, but kept around here so we can reuse the allocation.
    msgpack::sbuffer notificationBuf_;

    /// Same as notificationBuf_, but for streaming packets.
    msgpack::sbuffer streamingBuf_;
};
}

#endif

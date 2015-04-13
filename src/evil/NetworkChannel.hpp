#ifndef EVIL_NETWORKCHANNEL_HPP
#define EVIL_NETWORKCHANNEL_HPP

#include <cstdint>
#include <memory>
#include <string>
#include "azmq/socket.hpp"
#include "boost/asio/io_service.hpp"
#include "boost/log/sources/logger.hpp"
#include "evil/HardwareChannel.hpp"
#include "msgpack/sbuffer.hpp"

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

    uint16_t rpcPort() const { return rpcSocket_.port; }

private:
    NetworkChannel(boost::asio::io_service &ioService,
                   std::shared_ptr<HardwareChannel> hw);

    void receiveRpcCommand();
    void processRpcCommand(const char *data, size_t sizeBytes);

    void sendRpcSuccessResponse();
    template <typename T> void sendRpcSuccessResponse(T &&returnVal);
    void sendRpcErrorResponse(const std::string &errorMsg);

    template <typename... T>
    void sendNotification(const std::string &name, T &&... values);

    void sendStreamPacket(StreamIdx idx, const StreamPacket &packet);

    struct ZmqSocket {
        ZmqSocket(boost::asio::io_service &ioService, int type);
        azmq::socket socket;
        uint16_t port;
    };

    std::shared_ptr<HardwareChannel> hw_;

    ZmqSocket rpcSocket_;
    ZmqSocket notificationSocket_;
    std::vector<std::unique_ptr<ZmqSocket>> streamingSockets_;

    /// Whether an RPC request is currently processed. Used to determine whether
    /// an error needs to be sent during shutdown.
    bool currentlyProcessingRpc_;

    /// Buffer for constructing the RPC response. We only ever need once because
    /// of the strict request/reply model enforced by ZeroMQ.
    msgpack::sbuffer rpcResponseBuf_;

    /// The msgpack-rpc msgid of the last RPC request, for constructing the
    /// response (opaque blob chosen by the client).
    uint32_t lastRequestMsgId_;

    /// Buffer for sending notifications. Only ever used while sendNotification
    /// is executing, but kept around here so we can reuse the allocation.
    msgpack::sbuffer notificationBuf_;

    /// Same as notificationBuf_, but for streaming packets.
    msgpack::sbuffer streamingBuf_;

    boost::log::sources::logger log_;
};
}

#endif

#ifndef DEVIL_RPCINTERFACE_HPP
#define DEVIL_RPCINTERFACE_HPP

#include <functional>
#include <memory>
#include <string>
#include "devil/ZmqSocket.hpp"
#include "msgpack.hpp"

namespace msgpackrpc {
const unsigned request = 0;
const unsigned reply = 1;
const unsigned notification = 2;
}

namespace devil {
class RpcInterface : public std::enable_shared_from_this<RpcInterface> {
public:
    using RequestHandler =
        std::function<bool(const std::string &method, msgpack::object &params)>;
    static std::shared_ptr<RpcInterface>
    make(boost::asio::io_service &ioService, RequestHandler requestHandler) {
        return std::shared_ptr<RpcInterface>(
            new RpcInterface(ioService, requestHandler));
    }

    void start();
    void stop();

    uint16_t port() const { return socket_.port; }

    void sendErrorResponse(const std::string &errorMsg);
    void sendSuccessResponse();
    template <typename T> void sendSuccessResponse(T &&returnVal) {
        responseBuf_.clear();

        using Msg =
            msgpack::type::tuple<unsigned, unsigned, msgpack::type::nil, T>;
        msgpack::pack(responseBuf_, Msg(msgpackrpc::reply, lastRequestMsgId_,
                                        {}, std::forward<T>(returnVal)));

        sendResponseBuf();
    }

private:
    RpcInterface(boost::asio::io_service &ioService,
                 RequestHandler requestHandler);

    void sendResponseBuf();

    void nextRequest();

    ZmqSocket socket_;
    RequestHandler requestHandler_;

    /// Buffer for constructing the RPC response. We only ever need once because
    /// of the strict request/reply model enforced by ZeroMQ.
    msgpack::sbuffer responseBuf_;

    /// The msgpack-rpc msgid of the last RPC request, for constructing the
    /// response (opaque blob chosen by the client).
    uint32_t lastRequestMsgId_;
};
}

#endif

#ifndef DEVIL_RPCINTERFACE_HPP
#define DEVIL_RPCINTERFACE_HPP

#include <functional>
#include <memory>
#include <string>
#include "devil/ZmqSocket.hpp"
#include "msgpack.hpp"

/// msgpack-rpc message types.
namespace msgpackrpc {
const unsigned request = 0;
const unsigned reply = 1;
const unsigned notification = 2;
}

namespace devil {

/// \brief Manages a msgpack-rpc server on a ZeroMQ REP socket.
///
/// The implementation is fully non-blocking, using azmq.
///
/// ZeroMQ enforces a strict request/response/request/response/… sequence of
/// messages on REQ/REP socket pairs. This makes the interface for user of
/// this class very simple: The supplied callback is invoked whenever there is a
/// new request waiting. To reply to the latest request the user then calls one
/// of the public `send*Response()` functions, at which point the server
/// continues to asynchronously wait for the next request.
class RpcInterface : public std::enable_shared_from_this<RpcInterface> {
public:
    /// \brief Callback function for handling new msgpack-rpc requests. Should
    /// return true if the request was handled, false if the method was unknown.
    using RequestHandler =
        std::function<bool(const std::string &method, msgpack::object &params)>;

    /// \brief Constructs a new instance.
    ///
    /// The constructor itself is not public as the chosen strategy to deal with
    /// memory management in the face of the asynchronous callbacks relies on
    /// the lifetime being managed by std::shared_ptr.
    static std::shared_ptr<RpcInterface>
    make(boost::asio::io_service &ioService, RequestHandler requestHandler) {
        return std::shared_ptr<RpcInterface>(
            new RpcInterface(ioService, requestHandler));
    }

    /// \brief Starts asynchronously listening for device events.
    ///
    /// Invalid to call more than once, even after #stop() has been invoked.
    void start();

    /// \brief Signals the implementation to exit cleanly at the next possible
    /// point in time.
    ///
    /// Should not leave any active async operations behind so that boost::asio
    /// can be shut down cleanly afterwards.
    void stop();

    /// \brief Returns the TCP port assigned to the REP socket.
    uint16_t port() const { return socket_.port; }

    /// \brief Sends a msgpack-rpc error reply to the client.
    ///
    /// The error message is transmitted as a msgpack string in the error field
    /// of the reply struct.
    void sendErrorResponse(const std::string &errorMsg);

    /// \brief Sends a msgpack-rpc reply with a nil return value to the client.
    void sendSuccessResponse();

    /// \brief Sends a msgpack-rpc reply with the given return value to the
    /// client.
    ///
    /// The passed return value is converted to a msgpack in the default way.
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

    /// Sends the contents of responseBuf_ to the client and asynchronously
    /// continues to listen for requests.
    void sendResponseBuf();

    /// Asynchronously listens for the next request on the REP socket.
    void nextRequest();

    /// REP socket on which to listen for client requests.
    ZmqSocket socket_;

    /// Invoked for new request packets.
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

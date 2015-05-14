#include "devil/RpcInterface.hpp"

#include "azmq/message.hpp"
#include "boost/log/trivial.hpp"

using namespace boost::asio;
using namespace boost::log;
using namespace boost::system;

namespace devil {

RpcInterface::RpcInterface(io_service &ioService, RequestHandler requestHandler)
    : socket_{ioService, ZMQ_REP}, requestHandler_{requestHandler} {}

void RpcInterface::start() { nextRequest(); }

void RpcInterface::stop() {
    socket_.close();
    requestHandler_ = nullptr;
}

void RpcInterface::sendSuccessResponse() {
    responseBuf_.clear();

    using Msg = msgpack::type::tuple<unsigned, unsigned, msgpack::type::nil,
                                     msgpack::type::nil>;
    msgpack::pack(responseBuf_,
                  Msg(msgpackrpc::reply, lastRequestMsgId_, {}, {}));

    sendResponseBuf();
}

void RpcInterface::sendErrorResponse(const std::string &errorMsg) {
    responseBuf_.clear();

    using Msg = msgpack::type::tuple<unsigned, unsigned, const std::string &,
                                     msgpack::type::nil>;
    msgpack::pack(responseBuf_,
                  Msg(msgpackrpc::reply, lastRequestMsgId_, errorMsg, {}));

    sendResponseBuf();
}

void RpcInterface::sendResponseBuf() {
    auto self = shared_from_this();
    socket_.socket.async_send(buffer(responseBuf_.data(), responseBuf_.size()),
                              [this, self](const error_code &ec, size_t) {
                                  if (ec) {
                                      BOOST_LOG_TRIVIAL(warning)
                                          << "Error sending RPC response: "
                                          << ec.message();
                                  }
                                  nextRequest();
                              });
}

void RpcInterface::nextRequest() {
    auto self = shared_from_this();
    socket_.socket.async_receive([this, self](
        const error_code &ec, const azmq::message &msg, size_t) {
        if (ec == errc::operation_canceled) {
            // We are shutting down.
            return;
        }

        if (ec) {
            // Need to figure out when this happens, and how to respond
            // (such as not to break the ZeroMQ request/reply contract).
            BOOST_LOG_TRIVIAL(error)
                << "Error receiving RPC message, what to do? " << ec.message()
                << " (" << ec << ")";
            nextRequest();
            return;
        }

        if (msg.more()) {
            BOOST_LOG_TRIVIAL(info)
                << "Received unexpected multipart message on "
                   "RPC socket.";
            sendErrorResponse("Unexpected multipart message");
            return;
        }

        try {
            auto unp = msgpack::unpack(buffer_cast<const char *>(msg.buffer()),
                                       buffer_size(msg.buffer()));

            using Request = msgpack::type::tuple<unsigned, unsigned,
                                                 std::string, msgpack::object>;
            const auto request = unp.get().as<Request>();

            const auto type = request.get<0>();
            if (type != msgpackrpc::request) {
                BOOST_LOG_TRIVIAL(info)
                    << "Malformed RPC packet received: Invalid type: " << type;
                return;
            }

            lastRequestMsgId_ = request.get<1>();
            const auto method = request.get<2>();
            auto params = request.get<3>();

            if (requestHandler_(method, params)) return;

            BOOST_LOG_TRIVIAL(info)
                << "Malformed RPC packet received: Unknown method: " << method;
            sendErrorResponse("Unknown method invoked.");
            return;
        } catch (msgpack::unpack_error &e) {
            BOOST_LOG_TRIVIAL(info)
                << "Malformed RPC packet received: " << e.what();
            sendErrorResponse("Type mismatch.");
            return;
        } catch (std::bad_cast &e) {
            // msgpack throws bad_cast when we receive a valid msgpack object
            // but try to get() it as an mismatched type.
            BOOST_LOG_TRIVIAL(info)
                << "Malformed RPC packet received: " << e.what();
            sendErrorResponse("Type mismatch.");
            return;
        } catch (...) {
            sendErrorResponse("Internal error.");
            return;
        }

        assert(false &&
               "Should have replied to RPC message (enforced by ZeroMQ).");
    });
}
}

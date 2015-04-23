#include "evil/NetworkChannel.hpp"

#include <chrono>
#include "azmq/message.hpp"
#include "boost/log/common.hpp"
#include "msgpack.hpp"

using namespace boost::asio;
using namespace boost::log;
using namespace boost::system;
using namespace std::literals;

namespace evil {

namespace {
namespace methods {
const auto readRegister = "readRegister"s;
const auto modifyRegister = "modifyRegister"s;
const auto notificationPort = "notificationPort"s;
const auto streamPorts = "streamPorts"s;
const auto streamAcquisitionConfig = "streamAcquisitionConfig"s;
const auto setStreamAcquisitionConfig = "setStreamAcquisitionConfig"s;
}

namespace notifications {
const auto registerChanged = "registerChanged"s;
const auto streamAcquisitionConfigChanged = "streamAcquisitionConfigChanged"s;
const auto shutdown = "shutdown"s;
}

namespace msgpackrpc {
const unsigned request = 0;
const unsigned reply = 1;
const unsigned notification = 2;
}
}

NetworkChannel::NetworkChannel(io_service &ioService,
                               std::shared_ptr<HardwareChannel> hw)
    : hw_{std::move(hw)}, rpcSocket_{ioService, ZMQ_REP},
      notificationSocket_{ioService, ZMQ_PUB}, currentlyProcessingRpc_{false} {
    for (unsigned i = 0; i < hw_->streamCount(); ++i) {
        streamingSockets_.emplace_back(new ZmqSocket(ioService, ZMQ_PUB));
    }
}

void NetworkChannel::start() {
    auto self = shared_from_this();
    hw_->addShutdownCallback([this, self] {
        sendNotification(notifications::shutdown);
        if (currentlyProcessingRpc_) {
            sendRpcErrorResponse(
                "Hardware has been disconnected or server is shutting down.");
        }

        rpcSocket_.socket.cancel();
    });

    hw_->addRegisterChangeCallback([this, self](RegIdx idx, RegValue val) {
        sendNotification(notifications::registerChanged, idx, val);
    });

    for (unsigned streamIdx = 0; streamIdx < hw_->streamCount(); ++streamIdx) {
        // TODO: Do this only when there are actually subscribers for the stream
        // in question.
        hw_->setStreamPacketCallback(
            streamIdx, [this, self, streamIdx](const StreamPacket &packet) {
                sendStreamPacket(streamIdx, packet);
            });
    }

    receiveRpcCommand();
}

void NetworkChannel::stop() {
    // Just stop hw, we'll do our cleanup in the shutdown callback we registered
    // with it.
    hw_->stop();
}

void NetworkChannel::receiveRpcCommand() {
    currentlyProcessingRpc_ = false;

    auto self = shared_from_this();
    rpcSocket_.socket.async_receive(
        [this, self](const error_code &ec, const azmq::message &msg, size_t) {
            if (ec == errc::operation_canceled) {
                // We are shutting down.
                return;
            }

            currentlyProcessingRpc_ = true;

            if (ec) {
                // Need to figure out when this happens, and how to respond
                // (such as not to break the ZeroMQ request/reply contract).
                BOOST_LOG(log_) << "Error receiving RPC message, what to do? "
                                << ec.message() << " (" << ec << ")";
                receiveRpcCommand();
                return;
            }

            if (msg.more()) {
                BOOST_LOG(log_) << "Received unexpected multipart message on "
                                   "RPC socket.";
                sendRpcErrorResponse("Unexpected multipart message");
                return;
            }

            const auto msgBuf = msg.buffer();
            processRpcCommand(buffer_cast<const char *>(msgBuf),
                              buffer_size(msgBuf));
        });
}

void NetworkChannel::processRpcCommand(const char *data, size_t sizeBytes) {
    try {
        auto unp = msgpack::unpack(data, sizeBytes);

        using Request = msgpack::type::tuple<unsigned, unsigned, std::string,
                                             msgpack::object>;
        const auto request = unp.get().as<Request>();

        const auto type = request.get<0>();
        if (type != msgpackrpc::request) {
            BOOST_LOG(log_)
                << "Malformed RPC packet received: Invalid type: " << type;
            return;
        }

        lastRequestMsgId_ = request.get<1>();
        const auto method = request.get<2>();
        auto params = request.get<3>();

        using NoParams = msgpack::type::tuple<>;
        if (method == methods::readRegister) {
            const auto regIdx =
                std::get<0>(params.as<msgpack::type::tuple<RegIdx>>());
            if (!hw_->isValidRegister(regIdx)) {
                sendRpcErrorResponse("Invalid register index.");
                return;
            }
            sendRpcSuccessResponse(hw_->readRegister(regIdx));
            return;
        }

        if (method == methods::modifyRegister) {
            RegIdx regIdx;
            RegValue oldVal;
            RegValue newVal;
            std::tie(regIdx, oldVal, newVal) =
                params.as<msgpack::type::tuple<RegIdx, RegValue, RegValue>>();
            if (!hw_->isValidRegister(regIdx)) {
                sendRpcErrorResponse("Invalid register index.");
                return;
            }
            sendRpcSuccessResponse(hw_->modifyRegister(regIdx, oldVal, newVal));
            return;
        }

        if (method == methods::notificationPort) {
            params.as<NoParams>();
            sendRpcSuccessResponse(notificationSocket_.port);
            return;
        }

        if (method == methods::streamPorts) {
            params.as<NoParams>();
            std::vector<uint16_t> ports;
            ports.reserve(streamingSockets_.size());
            std::transform(streamingSockets_.begin(), streamingSockets_.end(),
                           std::back_inserter(ports),
                           [](const auto &a) { return a->port; });
            sendRpcSuccessResponse(ports);
            return;
        }

        if (method == methods::streamAcquisitionConfig) {
            params.as<NoParams>();
            if (streamingSockets_.empty()) {
                sendRpcErrorResponse("Hardware has no data streams.");
                return;
            }

            // What channel we use here does not matter.
            const auto config = hw_->streamAcquisitionConfig(0);
            sendRpcSuccessResponse(msgpack::type::tuple<double, unsigned>(
                config.timeSpan / 1s, config.sampleCount));

            return;
        }

        if (method == methods::setStreamAcquisitionConfig) {
            if (streamingSockets_.empty()) {
                sendRpcErrorResponse("Hardware has no data streams.");
                return;
            }

            double timeSpanSeconds;
            unsigned sampleCount;
            std::tie(timeSpanSeconds, sampleCount) =
                params.as<msgpack::type::tuple<double, unsigned>>();

            StreamAcquisitionConfig config{timeSpanSeconds * 1s, sampleCount};
            for (StreamIdx i = 0; i < hw_->streamCount(); ++i) {
                hw_->setStreamAcquisitionConfig(i, config);
            }

            sendNotification(notifications::streamAcquisitionConfigChanged,
                             timeSpanSeconds, sampleCount);

            sendRpcSuccessResponse();
            return;
        }

        BOOST_LOG(log_) << "Malformed RPC packet received: Unknown method: "
                        << method;
        sendRpcErrorResponse("Unknown method invoked.");
        return;
    } catch (msgpack::unpack_error &e) {
        BOOST_LOG(log_) << "Malformed RPC packet received: " << e.what();
        sendRpcErrorResponse("Type mismatch.");
        return;
    } catch (std::bad_cast &e) {
        // msgpack throws bad_cast when we receive a valid msgpack object
        // but try to get() it as an mismatched type.
        BOOST_LOG(log_) << "Malformed RPC packet received: " << e.what();
        sendRpcErrorResponse("Type mismatch.");
        return;
    } catch (...) {
        sendRpcErrorResponse("Internal error.");
        return;
    }

    assert(false && "Should have replied to RPC message (enforced by ZeroMQ).");
}

void NetworkChannel::sendRpcSuccessResponse() {
    rpcResponseBuf_.clear();

    using Msg = msgpack::type::tuple<unsigned, unsigned, msgpack::type::nil,
                                     msgpack::type::nil>;
    msgpack::pack(rpcResponseBuf_,
                  Msg(msgpackrpc::reply, lastRequestMsgId_, {}, {}));

    sendRpcResponseBuf();
}

template <typename T>
void NetworkChannel::sendRpcSuccessResponse(T &&returnVal) {
    rpcResponseBuf_.clear();

    using Msg = msgpack::type::tuple<unsigned, unsigned, msgpack::type::nil, T>;
    msgpack::pack(rpcResponseBuf_, Msg(msgpackrpc::reply, lastRequestMsgId_, {},
                                       std::forward<T>(returnVal)));

    sendRpcResponseBuf();
}

void NetworkChannel::sendRpcErrorResponse(const std::string &errorMsg) {
    rpcResponseBuf_.clear();

    using Msg = msgpack::type::tuple<unsigned, unsigned, const std::string &,
                                     msgpack::type::nil>;
    msgpack::pack(rpcResponseBuf_,
                  Msg(msgpackrpc::reply, lastRequestMsgId_, errorMsg, {}));

    sendRpcResponseBuf();
}

void NetworkChannel::sendRpcResponseBuf() {
    auto self = shared_from_this();
    rpcSocket_.socket.async_send(
        buffer(rpcResponseBuf_.data(), rpcResponseBuf_.size()),
        [this, self](const error_code &ec, size_t) {
            if (ec) {
                BOOST_LOG(log_)
                    << "Error sending RPC response: " << ec.message();
            }
            receiveRpcCommand();
        });
}

template <typename... T>
void NetworkChannel::sendNotification(const std::string &name, T &&... values) {
    notificationBuf_.clear();

    using Values = msgpack::type::tuple<T...>;
    using Msg = msgpack::type::tuple<unsigned, const std::string &, Values>;
    msgpack::pack(notificationBuf_, Msg(msgpackrpc::notification, name,
                                        Values(std::forward<T>(values)...)));

    notificationSocket_.socket.send(
        buffer(notificationBuf_.data(), notificationBuf_.size()));
}

void NetworkChannel::sendStreamPacket(StreamIdx idx,
                                      const StreamPacket &packet) {
    assert(idx < streamingSockets_.size() &&
           "Invalid streaming channel index.");

    streamingBuf_.clear();

    msgpack::packer<msgpack::sbuffer> p{streamingBuf_};

    p.pack_array(3);
    {
        p << msgpackrpc::notification;
        p << "streamPacket"s;
        p.pack_map(4);
        {
            p << "sampleIntervalSeconds"s;
            p << packet.sampleInterval / 1s;

            p << "sampleCount"s;
            p << packet.samples.size();

            p << "dataType"s;
            p << "int8"s;

            p << "samples"s;
            const auto sizeBytes =
                packet.samples.size() * sizeof(packet.samples[0]);
            p.pack_bin(sizeBytes);
            p.pack_bin_body(
                reinterpret_cast<const char *>(packet.samples.data()),
                sizeBytes);
        }
    }
    streamingSockets_[idx]->socket.send(
        buffer(streamingBuf_.data(), streamingBuf_.size()));
}

NetworkChannel::ZmqSocket::ZmqSocket(io_service &ioService, int type)
    : socket{ioService, type, true} {
    // Listen on any device, using a system-assigned port.
    socket.bind("tcp://*:*");

    // Parse the endpoint string to get the chosen port.
    auto e = socket.endpoint();
    port = std::stoi(e.substr(e.find_last_of(':') + 1));
}
}

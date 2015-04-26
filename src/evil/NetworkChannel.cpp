#include "evil/NetworkChannel.hpp"

#include <chrono>
#include "boost/log/trivial.hpp"

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

namespace extension_types {
const auto int8Array = 1;
}
}

NetworkChannel::NetworkChannel(io_service &ioService,
                               std::shared_ptr<HardwareChannel> hw)
    : ioService_{ioService}, hw_{std::move(hw)},
      notificationSocket_{ioService, ZMQ_PUB} {
    for (unsigned i = 0; i < hw_->streamCount(); ++i) {
        streamingSockets_.emplace_back(new ZmqSocket(ioService, ZMQ_PUB));
    }
}

void NetworkChannel::start() {
    auto self = shared_from_this();
    rpcInterface_ =
        RpcInterface::make(ioService_, [this, self](const std::string &method,
                                                    msgpack::object &params) {
            return processRpcCommand(method, params);
        });

    hw_->addShutdownCallback([this, self] {
        sendNotification(notifications::shutdown);
        rpcInterface_->stop();
    });

    hw_->addRegisterChangeCallback([this, self](RegIdx idx, RegValue val) {
        sendNotification(notifications::registerChanged, idx, val);
    });

    for (unsigned streamIdx = 0; streamIdx < hw_->streamCount(); ++streamIdx) {
        // TODO: Do this only when there are actually subscribers for the
        // stream in question.
        hw_->setStreamPacketCallback(
            streamIdx, [this, self, streamIdx](const StreamPacket &packet) {
                sendStreamPacket(streamIdx, packet);
            });
    }

    rpcInterface_->start();
}

void NetworkChannel::stop() {
    // Just stop hw, we'll do our cleanup in the shutdown callback we
    // registered with it.
    hw_->stop();
}

bool NetworkChannel::processRpcCommand(const std::string &method,
                                       msgpack::object &params) {
    using NoParams = msgpack::type::tuple<>;
    if (method == methods::readRegister) {
        const auto regIdx =
            std::get<0>(params.as<msgpack::type::tuple<RegIdx>>());
        if (!hw_->isValidRegister(regIdx)) {
            rpcInterface_->sendErrorResponse("Invalid register index.");
            return true;
        }
        rpcInterface_->sendSuccessResponse(hw_->readRegister(regIdx));
        return true;
    }

    if (method == methods::modifyRegister) {
        RegIdx regIdx;
        RegValue oldVal;
        RegValue newVal;
        std::tie(regIdx, oldVal, newVal) =
            params.as<msgpack::type::tuple<RegIdx, RegValue, RegValue>>();
        if (!hw_->isValidRegister(regIdx)) {
            rpcInterface_->sendErrorResponse("Invalid register index.");
            return true;
        }
        rpcInterface_->sendSuccessResponse(
            hw_->modifyRegister(regIdx, oldVal, newVal));
        return true;
    }

    if (method == methods::notificationPort) {
        params.as<NoParams>();
        rpcInterface_->sendSuccessResponse(notificationSocket_.port);
        return true;
    }

    if (method == methods::streamPorts) {
        params.as<NoParams>();
        std::vector<uint16_t> ports;
        ports.reserve(streamingSockets_.size());
        std::transform(streamingSockets_.begin(), streamingSockets_.end(),
                       std::back_inserter(ports),
                       [](const auto &a) { return a->port; });
        rpcInterface_->sendSuccessResponse(ports);
        return true;
    }

    if (method == methods::streamAcquisitionConfig) {
        params.as<NoParams>();
        if (streamingSockets_.empty()) {
            rpcInterface_->sendErrorResponse("Hardware has no data streams.");
            return true;
        }

        // What channel we use here does not matter.
        const auto config = hw_->streamAcquisitionConfig(0);
        rpcInterface_->sendSuccessResponse(
            msgpack::type::tuple<double, unsigned>(config.timeSpan / 1s,
                                                   config.sampleCount));

        return true;
    }

    if (method == methods::setStreamAcquisitionConfig) {
        if (streamingSockets_.empty()) {
            rpcInterface_->sendErrorResponse("Hardware has no data streams.");
            return true;
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

        rpcInterface_->sendSuccessResponse();
        return true;
    }
    return false;
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
        p.pack_array(1);
        {
            p.pack_map(3);
            {
                p << "sampleIntervalSeconds"s;
                p << packet.sampleInterval / 1s;

                p << "triggerOffset"s;
                p << packet.triggerOffset;

                p << "samples"s;
                const auto sizeBytes =
                    packet.samples.size() * sizeof(packet.samples[0]);
                p.pack_ext(sizeBytes, extension_types::int8Array);
                p.pack_ext_body(
                    reinterpret_cast<const char *>(packet.samples.data()),
                    sizeBytes);
            }
        }
    }
    streamingSockets_[idx]->socket.send(
        buffer(streamingBuf_.data(), streamingBuf_.size()));
}
}

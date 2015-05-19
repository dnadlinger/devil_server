#include "devil/NetworkChannel.hpp"

#include <chrono>
#include "boost/log/trivial.hpp"

using namespace boost::asio;
using namespace boost::log;
using namespace boost::system;
using namespace std::literals;

namespace devil {

namespace {
namespace methods {
const auto ping = "ping"s;
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
        for (auto &s : streamingMonitorSockets_) {
            s->cancel();
        }
        for (auto &s : streamingSockets_) {
            s->socket.close_monitor();
        }
    });

    hw_->addRegisterChangeCallback([this, self](RegIdx idx, RegValue val) {
        sendNotification(notifications::registerChanged, idx, val);
    });

    streamingMonitorSockets_.reserve(hw_->streamCount());
    streamingSubscriberCounts_.resize(hw_->streamCount(), 0);
    for (StreamIdx i = 0; i < hw_->streamCount(); ++i) {
        streamingMonitorSockets_.emplace_back(new azmq::socket(
            streamingSockets_[i]->socket.monitor(ioService_, ZMQ_EVENT_ALL)));
        nextMonitorEvent(i, *streamingMonitorSockets_.back());
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

    if (method == methods::ping) {
        params.as<NoParams>();
        rpcInterface_->sendSuccessResponse();
        return true;
    }

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

        // What channel we use here does not matter as we always set all to the
        // same config anyway.
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

namespace {
#pragma pack(1)
struct MonitorEvent {
    uint16_t type;
    uint32_t value;
};
#pragma options align = reset
}

void NetworkChannel::nextMonitorEvent(StreamIdx idx, azmq::socket &socket) {
    auto self = shared_from_this();
    socket.async_receive([this, self, idx, &socket](
        const error_code &ec, azmq::message &msg, size_t) {
        if (ec == errc::operation_canceled) {
            // We are shutting down.
            return;
        }

        if (ec) {
            BOOST_LOG_TRIVIAL(error) << "Error receiving streaming monitor "
                                        "event; should not happen unless there "
                                        "is a ZeroMQ-internal problem?";
            nextMonitorEvent(idx, socket);
            return;
        }

        if (!msg.more()) {
            BOOST_LOG_TRIVIAL(error) << "Streaming monitor event only consists "
                                        "of one part; should not happen unless "
                                        "there is a ZeroMQ-internal problem?";
            nextMonitorEvent(idx, socket);
            return;
        }

        MonitorEvent event;
        msg.buffer_copy(buffer(&event, sizeof(event)));

        if (event.type == ZMQ_EVENT_ACCEPTED) {
            addStreamSubscription(idx);
        } else if (event.type == ZMQ_EVENT_DISCONNECTED) {
            removeStreamSubscription(idx);
        } else {
            BOOST_LOG_TRIVIAL(info)
                << "Ignoring streaming socket monitor event: type = "
                << event.type << ", value = " << event.value;
        }

        socket.flush();
        nextMonitorEvent(idx, socket);
    });
}

void NetworkChannel::addStreamSubscription(StreamIdx idx) {
    auto &count = streamingSubscriberCounts_[idx];
    ++count;
    if (count == 1) {
        auto self = shared_from_this();
        hw_->setStreamPacketCallback(
            idx, [this, self, idx](const StreamPacket &packet) {
                sendStreamPacket(idx, packet);
            });
    }
}

void NetworkChannel::removeStreamSubscription(StreamIdx idx) {
    auto &count = streamingSubscriberCounts_[idx];
    --count;
    if (count == 0) {
        hw_->setStreamPacketCallback(idx, nullptr);
    }
}
}

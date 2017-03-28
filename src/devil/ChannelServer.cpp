#include "devil/ChannelServer.hpp"

#include <chrono>
#include "boost/log/trivial.hpp"

using namespace boost::asio;
using namespace boost::log;
using namespace boost::system;
using namespace std::literals;

namespace devil {

namespace {
/// The names of the msgpack-rpc methods for which requests are accepted. See
/// network-interface.md for details.
namespace methods {
const auto ping = "ping"s;
const auto readRegister = "readRegister"s;
const auto modifyRegister = "modifyRegister"s;
const auto notificationPort = "notificationPort"s;
const auto streamPorts = "streamPorts"s;
const auto streamAcquisitionConfig = "streamAcquisitionConfig"s;
const auto setStreamAcquisitionConfig = "setStreamAcquisitionConfig"s;
}

/// The names of the msgpack-rpc notifications sent to clients on the general
/// notification socket.
namespace notifications {
const auto registerChanged = "registerChanged"s;
const auto streamAcquisitionConfigChanged = "streamAcquisitionConfigChanged"s;
const auto shutdown = "shutdown"s;
}

/// Enumerates the msgpack extension type tages used in the interface.
namespace extension_types {
const auto int8Array = 1;
}
}

ChannelServer::ChannelServer(io_service &ioService, std::shared_ptr<Channel> hw)
    : ioService_{ioService}, hw_{std::move(hw)},
      notificationSocket_{ioService, ZMQ_PUB} {
    for (unsigned i = 0; i < hw_->streamCount(); ++i) {
        streamingSockets_.emplace_back(new ZmqSocket(ioService, ZMQ_PUB));
    }
}

void ChannelServer::start() {
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
        // In theory, we would want to only subscribes to events we care about
        // instead of ZMQ_EVENT_ALL. However, there do not seem to be any other
        // events emitted in normal use, and listening to all lets us figure out
        // if we also need to take others into consideration based on the logs.
        streamingMonitorSockets_.emplace_back(new azmq::socket(
            streamingSockets_[i]->socket.monitor(ioService_, ZMQ_EVENT_ALL)));
        nextMonitorEvent(i, *streamingMonitorSockets_.back());
    }

    rpcInterface_->start();
}

void ChannelServer::stop() {
    // Just stop hw, we'll do our cleanup in the shutdown callback we
    // registered with it.
    hw_->stop();
}

bool ChannelServer::processRpcCommand(const std::string &method,
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

        // Which channel we use here does not matter as we always set all to the
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

        for (StreamIdx i = 0; i < hw_->streamCount(); ++i) {
            auto config = hw_->streamAcquisitionConfig(i);
            config.timeSpan = timeSpanSeconds * 1s;
            config.sampleCount = sampleCount;
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
void ChannelServer::sendNotification(const std::string &name, T &&... values) {
    notificationBuf_.clear();

    using Values = msgpack::type::tuple<T...>;
    using Msg = msgpack::type::tuple<unsigned, const std::string &, Values>;
    msgpack::pack(notificationBuf_, Msg(msgpackrpc::notification, name,
                                        Values(std::forward<T>(values)...)));

    notificationSocket_.socket.send(
        buffer(notificationBuf_.data(), notificationBuf_.size()));
}

void ChannelServer::sendStreamPacket(StreamIdx idx,
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
/// A socket monitor message. Format as per the ZeroMQ API docs.
#pragma pack(1)
struct MonitorEvent {
    uint16_t type;
    uint32_t value;
};
#pragma options align = reset
}

void ChannelServer::nextMonitorEvent(StreamIdx idx, azmq::socket &socket) {
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
            // This might be triggered if the ZeroMQ API is changed in the
            // future. Have a look at the API docs to see if we need to handle
            // that event type then. Currently, all the other events should not
            // be triggered during normal operation of the server.
            BOOST_LOG_TRIVIAL(info)
                << "Ignoring streaming socket monitor event: type = "
                << event.type << ", value = " << event.value;
        }

        socket.flush();
        nextMonitorEvent(idx, socket);
    });
}

void ChannelServer::addStreamSubscription(StreamIdx idx) {
    auto &count = streamingSubscriberCounts_[idx];
    ++count;

    // If this is the first subscriber, register the streaming callback, which
    // will also cause hardware polling to start.
    if (count == 1) {
        auto self = shared_from_this();
        hw_->setStreamPacketCallback(
            idx, [this, self, idx](const StreamPacket &packet) {
                sendStreamPacket(idx, packet);
            });
    }

    updateStreamDutyCycles();
}

void ChannelServer::removeStreamSubscription(StreamIdx idx) {
    auto &count = streamingSubscriberCounts_[idx];
    --count;

    // If this was the last subscriber, remove the streaming callback so that
    // hardware polling can be stopped.
    if (count == 0) {
        hw_->setStreamPacketCallback(idx, nullptr);
    }

    updateStreamDutyCycles();
}

void ChannelServer::updateStreamDutyCycles() {
    const auto maxSubs = *std::max_element(streamingSubscriberCounts_.begin(),
                                           streamingSubscriberCounts_.end());
    // If there are multiple subscribers for at least some of the channels, only
    // update the ones with one subscriber a fraction of the time. This is a
    // heuristic to deal with a single logging client per network that logs e.g.
    // error signal and output, while the user is only interested in one of them
    // during manual setup. If using a long acquisition time span for a slow
    // controlled device, streaming both at the same rate would lead to
    // undesirable lag. Ideally, the type of client should be indicated somehow
    // in the stream subscription, but as the subscription is currently simply
    // established by opening a connection to the streaming port, this is the
    // best we can do for now.
    for (size_t i = 0; i < streamingSubscriberCounts_.size(); ++i) {
        auto config = hw_->streamAcquisitionConfig(i);
        config.dutyCycle =
            (maxSubs > 1 && streamingSubscriberCounts_[i] == 1) ? 8 : 1;
        hw_->setStreamAcquisitionConfig(i, config);
    }
}
}

#include "fliquer.hpp"

#include <algorithm>
#include "boost/asio/steady_timer.hpp"
#include "boost/log/common.hpp"
#include "msgpack.hpp"
#include "msgpack/object_fwd.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace {
const unsigned msgType = 2;
namespace methods {
const auto enumerate = "enumerate"s;
const auto resources = "resources"s;
};
}

namespace fliquer {

Node::Node(io_service &ioService,
           Node::NewRemoteResourceCallback newRemoteResourceCallback,
           uint16_t port)
    : newRemoteResourceCallback_{newRemoteResourceCallback}, port_{port},
      udpSocket_{ioService} {
    // We need to set reuse_address before binding the socket, but for this it
    // already needs to be open. Thus, we explicitly need to call open().
    udpSocket_.open(ip::udp::v4());
    udpSocket_.set_option(socket_base::reuse_address(true));
    udpSocket_.set_option(socket_base::broadcast(true));
    udpSocket_.bind(ip::udp::endpoint(ip::udp::v4(), port));
}

void Node::start() {
    receiveUdpPacket();
    if (newRemoteResourceCallback_) {
        broadcastEnumerationRequest();
    }
}

void Node::stop() { udpSocket_.close(); }

void Node::addLocalResource(const Resource &resource) {
    assert(std::find(localResources_.begin(), localResources_.end(),
                     resource) == localResources_.end() &&
           "Resource already added to the list.");
    localResources_.push_back(resource);

    auto buf = std::make_shared<msgpack::sbuffer>();

    using Msg = msgpack::type::tuple<unsigned, const std::string &,
                                     msgpack::type::tuple<const Resource &>>;

    msgpack::type::tuple<const Resource &> params{resource};
    msgpack::pack(*buf, Msg(msgType, methods::resources, params));

    const ip::udp::endpoint broadcast{ip::address_v4::broadcast(), port_};

    auto self = shared_from_this();
    udpSocket_.async_send_to(buffer(buf->data(), buf->size()), broadcast,
                             [this, self, buf](const error_code &err, size_t) {
                                 if (err) {
                                     if (err == errc::bad_file_descriptor) {
                                         // The socket has been closed by
                                         // stop(), exit the callback
                                         // chain.
                                         return;
                                     }
                                     BOOST_LOG(log_)
                                         << "Failed to send broadcast: " << err;
                                 }
                             });
}

bool Node::removeLocalResource(const Resource &resource) {
    auto it =
        std::find(localResources_.begin(), localResources_.end(), resource);
    if (it == localResources_.end()) return false;

    localResources_.erase(it);
    return true;
}

void Node::broadcastEnumerationRequest() {
    auto requestBuf = std::make_shared<msgpack::sbuffer>();

    using Msg = msgpack::type::tuple<unsigned, const std::string &,
                                     msgpack::type::tuple<>>;
    msgpack::pack(*requestBuf, Msg(msgType, methods::enumerate, {}));

    const ip::udp::endpoint broadcast{ip::address_v4::broadcast(), port_};

    auto self = shared_from_this();
    udpSocket_.async_send_to(
        buffer(requestBuf->data(), requestBuf->size()), broadcast,
        [this, self, requestBuf](const error_code &err, size_t) {
            if (err) {
                if (err == errc::bad_file_descriptor) {
                    // The socket has been closed by stop(), exit the callback
                    // chain.
                    return;
                }
                BOOST_LOG(log_) << "Failed to send broadcast: " << err.message()
                                << " (" << err << ")";
            }
        });
}

void Node::receiveUdpPacket() {
    auto self = shared_from_this();
    udpSocket_.async_receive_from(
        buffer(receiveBuf_.data(), receiveBuf_.size()), receiveSender_,
        [this, self](const error_code &err, size_t receivedBytes) {
            if (err) {
                if (err == errc::bad_file_descriptor ||
                    err == errc::operation_canceled) {
                    // The socket has been closed by stop(), exit the callback
                    // chain.
                    return;
                }
                BOOST_LOG(log_) << "Error while receiving UDP packet: " << err;
            } else {
                actOnUdpPacket(receivedBytes);
            }
            receiveUdpPacket();
        });
}

void Node::actOnUdpPacket(size_t sizeBytes) {
    try {
        auto unp = msgpack::unpack(receiveBuf_.begin(), sizeBytes);

        using Msg =
            msgpack::type::tuple<unsigned, std::string, msgpack::object>;
        const auto msg = unp.get().as<Msg>();

        const auto type = msg.get<0>();
        if (type != msgType) {
            BOOST_LOG(log_)
                << "Malformed UDP packet received: Invalid type: " << type;
            return;
        }

        const auto method = msg.get<1>();
        if (method == methods::enumerate) {
            replyWithLocalResources();
        } else if (method == methods::resources) {
            if (!newRemoteResourceCallback_) {
                // We are not interested in remote resources, so just ignore
                // resource packets.
                return;
            }

            const auto resources = msg.get<2>().as<std::vector<Resource>>();
            const auto addr = receiveSender_.address();

            std::vector<RemoteResource> rrs;
            rrs.reserve(resources.size());
            for (const auto &r : resources) {
                rrs.emplace_back(addr, r);
            }

            newRemoteResourceCallback_(std::move(rrs));
        } else {
            BOOST_LOG(log_)
                << "Malformed UDP packet received: Unknown method: " << method;
        }
    } catch (msgpack::unpack_error &e) {
        BOOST_LOG(log_) << "Malformed UDP packet received: " << e.what();
    } catch (std::bad_cast &e) {
        // msgpack throws bad_cast when we receive a valid msgpack object
        // but
        // try to get() it as an mismatched type.
        BOOST_LOG(log_) << "Malformed UDP packet received: " << e.what();
    }
}

void Node::replyWithLocalResources() {
    // No need sending an empty resource packet.
    if (localResources_.empty()) return;

    auto buf = std::make_shared<msgpack::sbuffer>();

    using Msg = msgpack::type::tuple<unsigned, std::string,
                                     const std::vector<Resource> &>;
    msgpack::pack(*buf, Msg(msgType, methods::resources, localResources_));

    auto self = shared_from_this();
    udpSocket_.async_send_to(buffer(buf->data(), buf->size()), receiveSender_,
                             [this, self, buf](const error_code &err, size_t) {
                                 if (err) {
                                     if (err == errc::bad_file_descriptor) {
                                         // The socket has been closed by
                                         // stop(), exit the callback
                                         // chain.
                                         return;
                                     }
                                     BOOST_LOG(log_)
                                         << "Failed to send reply: " << err;
                                 }
                             });
}
}

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
const auto allLocal = "all"s;
const auto enumerate = "enumerate"s;
const auto newLocal = "new"s;
};
}

namespace fliquer {

Node::Node(io_service &ioService,
           Node::NewRemoteResourceCallback newRemoteResourceCallback,
           uint16_t port)
    : ioService_{ioService},
      newRemoteResourceCallback_{newRemoteResourceCallback}, port_{port},
      udpSocket_{ioService, ip::udp::endpoint(ip::udp::v4(), port)} {
    udpSocket_.set_option(ip::udp::socket::reuse_address(true));
    udpSocket_.set_option(socket_base::broadcast(true));
}

void Node::addLocalResource(const Resource &resource) {
    assert(std::find(localResources_.begin(), localResources_.end(),
                     resource) == localResources_.end() &&
           "Resource already added to the list.");
    localResources_.push_back(resource);

    auto buf = std::make_shared<msgpack::sbuffer>();

    using Msg = msgpack::type::tuple<unsigned, const std::string &,
                                     msgpack::type::tuple<const Resource &>>;

    msgpack::type::tuple<const Resource &> params{resource};
    msgpack::pack(*buf, Msg(msgType, methods::newLocal, params));

    const ip::udp::endpoint broadcast{ip::address_v4::broadcast(), port_};

    auto self = shared_from_this();
    udpSocket_.async_send_to(buffer(buf->data(), buf->size()), broadcast,
                             [this, self, buf](const error_code &err, size_t) {
                                 if (err) {
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

namespace {
class Enumerator : public std::enable_shared_from_this<Enumerator> {
public:
    Enumerator(io_service &i, std::chrono::seconds t, uint16_t port,
               Node::RemoteResourcesCallback callback)
        : socket_{ip::udp::socket{i, ip::udp::endpoint{ip::udp::v4(), 0}}},
          timeout_{i, t}, port_{port}, callback_{callback} {
        socket_.set_option(socket_base::broadcast(true));
    }

    void perform() {
        auto self = shared_from_this();

        // Install the timeout that stops waiting for responses.

        timeout_.async_wait([this, self](const error_code &) {
            socket_.close();
            callback_(resources_);
        });

        // Broadcast enumeration request.

        using Msg = msgpack::type::tuple<unsigned, const std::string &,
                                         msgpack::type::tuple<>>;
        msgpack::pack(requestBuf_, Msg(msgType, methods::enumerate, {}));

        const ip::udp::endpoint broadcast{ip::address_v4::broadcast(), port_};

        socket_.async_send_to(
            buffer(requestBuf_.data(), requestBuf_.size()), broadcast,
            [this, self](const error_code &err, size_t) {
                if (err) {
                    BOOST_LOG(log_)
                        << "Failed to send broadcast: " << err.message() << " ("
                        << err << ")";
                }
            });
    }

private:
    void receive() {
        auto self = shared_from_this();
        socket_.async_receive_from(
            buffer(receiveBuf_.data(), receiveBuf_.size()), lastSender_,
            [this, self](const error_code &err, size_t sizeBytes) {
                if (err) {
                    BOOST_LOG(log_)
                        << "Error while receiving UDP packet: " << err;
                } else {
                    try {
                        auto unp =
                            msgpack::unpack(receiveBuf_.begin(), sizeBytes);

                        using Msg = msgpack::type::tuple<unsigned, std::string,
                                                         std::vector<Resource>>;
                        auto msg = unp.get().as<Msg>();

                        auto type = msg.get<0>();
                        if (type != msgType) {
                            BOOST_LOG(log_) << "Malformed enumeration "
                                               "response: Invalid "
                                               "type: " << type;
                        } else {
                            auto method = msg.get<1>();
                            if (method != methods::allLocal) {
                                BOOST_LOG(log_)
                                    << "Malformed enumeration response: "
                                       "Unexpected method string.";
                            } else {
                                for (auto &r : msg.get<2>()) {
                                    resources_.emplace_back(
                                        lastSender_.address(), r);
                                }
                            }
                        }
                    } catch (msgpack::unpack_error &e) {
                        BOOST_LOG(log_)
                            << "Malformed enumeration response: " << e.what();
                    } catch (std::bad_cast &e) {
                        // msgpack throws bad_cast when we receive a valid
                        // msgpack object but
                        // try to get() it as an mismatched type.
                        BOOST_LOG(log_)
                            << "Malformed enumeration response: " << e.what();
                    }
                }
                receive();
            });
    }

    ip::udp::socket socket_;
    msgpack::sbuffer requestBuf_;

    steady_timer timeout_;
    uint16_t port_;

    ip::udp::endpoint lastSender_;
    std::array<char, maxUdpPacketSize> receiveBuf_;
    std::vector<RemoteResource> resources_;

    Node::RemoteResourcesCallback callback_;

    boost::log::sources::logger log_;
};
}

void Node::enumerateRemoteResources(std::chrono::seconds timeout,
                                    Node::RemoteResourcesCallback callback) {
    std::make_shared<Enumerator>(ioService_, timeout, port_, callback)
        ->perform();
}

void Node::receiveUdpPacket() {
    auto self = shared_from_this();
    udpSocket_.async_receive_from(
        buffer(receiveBuf_.data(), receiveBuf_.size()), receiveSender_,
        [this, self](const error_code &err, size_t receivedBytes) {
            if (err) {
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

        using Msg = msgpack::type::tuple<unsigned, std::string,
                                         std::vector<msgpack::object>>;
        auto msg = unp.get().as<Msg>();

        auto type = msg.get<0>();
        if (type != msgType) {
            BOOST_LOG(log_)
                << "Malformed UDP packet received: Invalid type: " << type;
            return;
        }

        auto method = msg.get<1>();
        if (method == methods::enumerate) {
            if (!msg.get<2>().empty()) {
                BOOST_LOG(log_) << "Malformed UDP packet received: "
                                   "No arguments excepted.";
                return;
            }
            replyWithLocalResources();
        } else if (method == methods::newLocal) {
            const auto &params = msg.get<2>();
            if (params.size() != 1) {
                BOOST_LOG(log_) << "Malformed UDP packet received: "
                                   "One argument excepted.";
                return;
            }
            newRemoteResourceCallback_(
                {receiveSender_.address(), params[0].as<Resource>()});
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
    auto buf = std::make_shared<msgpack::sbuffer>();

    using Msg = msgpack::type::tuple<unsigned, std::string,
                                     const std::vector<Resource> &>;
    msgpack::pack(*buf, Msg(msgType, methods::allLocal, localResources_));

    auto self = shared_from_this();
    udpSocket_.async_send_to(buffer(buf->data(), buf->size()), receiveSender_,
                             [this, self, buf](const error_code &err, size_t) {
                                 if (err) {
                                     BOOST_LOG(log_)
                                         << "Failed to send reply: " << err;
                                 }
                             });
}
}

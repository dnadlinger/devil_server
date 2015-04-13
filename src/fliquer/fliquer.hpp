#ifndef FLIQUER_NODE_H
#define FLIQUER_NODE_H

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "boost/asio/io_service.hpp"
#include "boost/asio/ip/udp.hpp"
#include "boost/log/sources/logger.hpp"
#include "msgpack/adaptor/define.hpp"
#include "msgpack/sbuffer.hpp"

namespace fliquer {

struct SemVer {
    unsigned major = 0;
    unsigned minor = 0;
    unsigned patch = 0;
    std::string preRelease;
    std::string buildMetadata;

    MSGPACK_DEFINE(major, minor, patch, preRelease, buildMetadata);

    bool operator==(const SemVer &rhs) const {
        return major == rhs.major && minor == rhs.minor && patch == rhs.patch &&
               preRelease == rhs.preRelease &&
               buildMetadata == rhs.buildMetadata;
    }
};

/// \brief A resource that can be discovered via Fliquer.
struct Resource {
    /// \brief A string identifying the type of the resource.
    ///
    /// Since this is used by clients to figure out how to handle the resource,
    /// it should be reasonably detailed to ensure uniqueness (for example,
    /// "tiqi.evil.pidchannel").
    std::string type;

    /// \brief Some kind of ID that should unique to the actual resource in the
    /// lab.
    ///
    /// For a hardware device, this might be some kind of serial number (e.g. of
    /// the FTDI USB chip).
    std::string id;

    /// \brief User-readable name of the resource to display in UIs, etc.
    std::string displayName;

    /// \brief Version of the software/interface the resource is running.
    ///
    /// This should follow [semver guidelines](http://semver.org/), where
    /// backwards-incompatible protocl changes are represented by major version
    /// jumps.
    ///
    /// Additional information such as build date can be included as metadata.
    SemVer version;

    /// \brief The network port of the main RPC interface to the resource.
    ///
    /// The exact meaning of this (TCP/UDP, protocol, …) depends on the type of
    /// the resource.
    uint16_t port;

    MSGPACK_DEFINE(type, id, displayName, version, port);

    bool operator==(const Resource &rhs) const {
        return type == rhs.type && id == rhs.id &&
               displayName == rhs.displayName && version == rhs.version &&
               port == rhs.port;
    }
};

using RemoteResource = std::pair<boost::asio::ip::address, Resource>;

enum { maxUdpPacketSize = 65507 };

class Node : public std::enable_shared_from_this<Node> {
public:
    using NewRemoteResourceCallback =
        std::function<void(const std::vector<RemoteResource> &)>;

    static std::shared_ptr<Node>
    make(boost::asio::io_service &ioService,
         NewRemoteResourceCallback newRemoteResourceCallback = nullptr,
         uint16_t port = 8474) {
        return std::shared_ptr<Node>(
            new Node(ioService, newRemoteResourceCallback, port));
    }

    void start();
    void stop();

    void addLocalResource(const Resource &resource);
    bool removeLocalResource(const Resource &resource);

private:
    Node(boost::asio::io_service &ioService,
         NewRemoteResourceCallback newRemoteResourceCallback, uint16_t port);

    void broadcastEnumerationRequest();
    void receiveUdpPacket();
    void actOnUdpPacket(size_t sizeBytes);
    void replyWithLocalResources();

    NewRemoteResourceCallback newRemoteResourceCallback_;
    uint16_t port_;

    boost::asio::ip::udp::socket udpSocket_;
    std::vector<Resource> localResources_;

    boost::asio::ip::udp::endpoint receiveSender_;
    std::array<char, maxUdpPacketSize> receiveBuf_;

    boost::log::sources::logger log_;
};
}

#endif

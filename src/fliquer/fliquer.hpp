#ifndef FLIQUER_NODE_H
#define FLIQUER_NODE_H

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <ostream>
#include <memory>
#include <string>
#include <vector>

#include "boost/asio/io_service.hpp"
#include "boost/asio/ip/udp.hpp"
#include "boost/log/sources/logger.hpp"
#include "msgpack/adaptor/define.hpp"
#include "msgpack/object.hpp"
#include "msgpack/sbuffer.hpp"

/// \brief An implementation of the Fliquer protocol for Finding Lab Instruments
/// for Quantum Experiments.
namespace fliquer {

/// \brief A [SemVer](http://semver.org/) version tuple.
///
/// Serialized to msgpack as a flat list.
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

/// \brief Prints a SemVer object to an ostream.
///
/// Empty pre-release or build metadata fields are omitted.
std::ostream &operator<<(std::ostream &str, const SemVer &s);

/// \brief A resource that can be discovered via Fliquer.
///
/// Note that this does not include an IP address. This is implicitly
/// represented by the sender field of the UDP packets making up the protocol.
///
/// Serialized to msgpack as a flat list.
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

    /// \brief Reserved to facilitate future protocol tweaks.
    ///
    /// Simply ignored and defaulted to nil by Fliquer 1.0 nodes.
    msgpack::object reserved;

    MSGPACK_DEFINE(type, id, displayName, version, port, reserved);

    bool operator==(const Resource &rhs) const {
        return type == rhs.type && id == rhs.id &&
               displayName == rhs.displayName && version == rhs.version &&
               port == rhs.port && reserved == rhs.reserved;
    }
};

/// \brief Represents a Resource at a given IP address.
using RemoteResource = std::pair<boost::asio::ip::address, Resource>;

/// \brief The maximum size of a single UDP packet.
///
/// The actual maximum varies wildly between different operating systems.
/// Currently used only for sizing the receive buffers. In the future, the reply
/// packets should be split up automatically to support nodes with a large
/// number of resources.
enum { maxUdpPacketSize = 65507 };

/// \brief A node in a Fliquer network that can broadcast its own resources and
/// enumerate the network for existing resources.
///
/// Listens on all IPv4 network interfaces and sends
///
/// Note: On Linux, broadcasting fails if there is no explicit network interface
/// with a broadcast interface configured. In other words, Linux will not
/// default to simply sending broadcast packets to the local host when there are
/// no other connections. Keep this in mind when developing on mobile PCs/… that
/// might not have a network connection all the.
class Node : public std::enable_shared_from_this<Node> {
public:
    using NewRemoteResourceCallback =
        std::function<void(const std::vector<RemoteResource> &)>;

    /// \brief Constructs a new instance.
    ///
    /// The constructor itself is not public as the chosen strategy to deal with
    /// memory management in the face of the asynchronous callbacks relies on
    /// the lifetime being managed by std::shared_ptr.
    ///
    /// \param newRemoteResourceCallback Callback to invoke for newly discovered
    ///     remote resources. Defaults to null, which causes the network not to
    ///     be enumerated at all.
    /// \param port The UDP port to use for communication.
    static std::shared_ptr<Node>
    make(boost::asio::io_service &ioService,
         NewRemoteResourceCallback newRemoteResourceCallback = nullptr,
         uint16_t port = 8474) {
        return std::shared_ptr<Node>(
            new Node(ioService, newRemoteResourceCallback, port));
    }

    /// \brief Announces the local resources on the network and starts
    /// asynchronously listening for messages from other nodes.
    ///
    /// Invalid to call more than once on a single instance, even after #stop()
    /// has been invoked.
    void start();

    /// \brief Signals the implementation to exit cleanly at the next possible
    /// point in time.
    ///
    /// Should not leave any active async operations behind so that boost::asio
    /// can be shut down cleanly afterwards.
    void stop();

    /// \brief Adds a new local resource to be advertised on the Fliquer
    /// network.
    void addLocalResource(const Resource &resource);

    /// \brief Removes a previously added local resource.
    ///
    /// \returns `true` if the resource had been registered previously, `false`
    /// if not.
    bool removeLocalResource(const Resource &resource);

private:
    Node(boost::asio::io_service &ioService,
         NewRemoteResourceCallback newRemoteResourceCallback, uint16_t port);

    void broadcastEnumerationRequest();
    void receiveUdpPacket();
    void actOnUdpPacket(size_t sizeBytes);
    void replyWithLocalResources();

    NewRemoteResourceCallback newRemoteResourceCallback_;
    const uint16_t port_;

    boost::asio::ip::udp::socket udpSocket_;
    std::vector<Resource> localResources_;

    /// Sender of the last received UDP message.
    boost::asio::ip::udp::endpoint receiveSender_;

    /// Buffer to receive UDP data into.
    std::array<char, maxUdpPacketSize> receiveBuf_;

    boost::log::sources::logger log_;
};
}

#endif

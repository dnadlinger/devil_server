#include "Server.hpp"

#include "boost/log/trivial.hpp"
#include "evil/Evil2Adapter.hpp"

using namespace boost::asio;
using namespace boost::log;
using namespace boost::system;

namespace evil {

Server::Server(io_service &ioService, ChannelNameMap channelNames)
    : ioService_{ioService}, channelNames_{std::move(channelNames)},
      deviceObserver_{DeviceObserver::make(
          ioService,
          [&](std::string path, std::string serial) {
              BOOST_LOG_TRIVIAL(info) << "EVIL connected at " << path
                                      << " (serial: " << serial << ")";

              auto conn = SerialConnection::make(ioService_, path);
              activeDevices_.push_back(conn);

              // Remove the connection from the list when it shuts down so the
              // memory can eventually be reclaimed.
              auto self = shared_from_this();
              conn->addShutdownCallback([this, self, conn]() {
                  auto begin = activeDevices_.begin();
                  auto end = activeDevices_.end();
                  auto it = std::find(begin, end, conn);
                  assert(it != end && "Connection not registered, shutdown "
                                      "callback invoked twice?");
                  activeDevices_.erase(it);
              });

              // Initialize the device.
              conn->start([this, self, conn, path, serial](
                  uint16_t versionMajor, uint8_t versionMinor) {
                  registerDevice(path, conn, serial, versionMajor,
                                 versionMinor);
              });
          },
          [&](std::string path, std::string serial) {
              // Right now, we just let the serial read fail and clean up our
              // side of things in the SerialConnection shutdown callback.
              (void)path;
              (void)serial;
          })},
      fliquer_{fliquer::Node::make(ioService)} {}

void Server::start() {
    deviceObserver_->start();
    fliquer_->start();
}

void Server::stop() {
    deviceObserver_->stop();
    fliquer_->stop();

    for (auto &conn : activeDevices_) {
        conn->stop();
    }
}

void Server::registerDevice(const std::string &path,
                            std::shared_ptr<SerialConnection> conn,
                            const std::string &serial, uint16_t versionMajor,
                            uint8_t versionMinor) {
    BOOST_LOG_TRIVIAL(info) << "Established connection to EVIL at " << path
                            << ": version " << +versionMajor << '.'
                            << +versionMinor;
    const auto announce =
        [&](const NetworkChannel &chan, std::string serialExt = "") {
            auto fullSerial = serial + serialExt;

            fliquer::Resource r;
            r.type = "tiqi.evil.channel";
            r.id = fullSerial;
            r.port = chan.rpcPort();

            fliquer::SemVer v;
            v.major = versionMajor;
            v.minor = versionMinor;
            r.version = v;

            const auto prettyName = channelNames_[fullSerial];
            r.displayName = prettyName.empty() ? fullSerial : prettyName;

            fliquer_->addLocalResource(r);
        };

    if (versionMajor == 1 || versionMajor == 3) {
        auto chan = NetworkChannel::make(ioService_, conn);
        chan->start();
        announce(*chan);
    } else if (versionMajor == 2) {
        auto a = std::make_shared<Evil2Adapter>(conn, Evil2Adapter::Channel::a);
        auto chanA = NetworkChannel::make(ioService_, a);
        chanA->start();
        announce(*chanA, ":A");

        auto b = std::make_shared<Evil2Adapter>(conn, Evil2Adapter::Channel::b);
        auto chanB = NetworkChannel::make(ioService_, b);
        chanB->start();
        announce(*chanB, ":B");
    } else {
        BOOST_LOG_TRIVIAL(warning) << "Do not know how to handle version "
                                   << versionMajor;
    }
}
}

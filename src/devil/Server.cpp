#include "Server.hpp"

#include <chrono>
#include <iomanip>
#include "boost/log/trivial.hpp"
#include "devil/DualChannelAdapter.hpp"
#include "msgpack.hpp"

using namespace boost::asio;
using namespace boost::log;
using namespace boost::system;
using namespace std::literals;

namespace {
std::string buildTimestamp() {
    std::tm tm;
    std::stringstream source;
    source << __DATE__ << " " << __TIME__;

    // Sigh… The libstdc++ shipped with 4.9 does not have get_time or put_time
    // yet.
    // source >> std::get_time(&tm, "%b %d %Y %H:%M:%S");
    strptime(source.str().c_str(), "%b %d %Y %H:%M:%S", &tm);

    // result << std::put_time(&tm, "%Y%m%d%H%M");
    std::array<char, 13> result;
    strftime(result.data(), result.size(), "%Y%m%d%H%M", &tm);
    return {result.data(), result.size()};
}

const auto statUpdateInterval = 1s;
}

namespace devil {

Server::Server(io_service &ioService, Server::Config config)
    : ioService_{ioService}, config_(std::move(config)),
      deviceObserver_{DeviceObserver::make(
          ioService, config_.id,
          [&](std::string path, std::string serial) {
              BOOST_LOG_TRIVIAL(info) << path << ": EVIL connected, serial '"
                                      << serial << "'";

              auto conn = SerialChannel::make(ioService_, path,
                                              config_.minLongStreamIntervalReg,
                                              performanceCounters_);
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
              // side of things in the SerialChannel shutdown callback.
              (void)path;
              (void)serial;
          })},
      fliquer_{fliquer::Node::make(ioService)},
      performanceCounters_{std::make_shared<PerformanceCounters>()},
      performanceStatsUpdateTimer_{ioService},
      managementInterface_{RpcInterface::make(
          ioService, [&](const std::string &method, msgpack::object & params) {
              if (method == "perfStats") {
                  params.as<msgpack::type::tuple<>>();
                  managementInterface_->sendSuccessResponse(performanceStats_);
                  return true;
              }
              return false;
          })} {}

void Server::start() {
    deviceObserver_->start();
    fliquer_->start();
    managementInterface_->start();

    announceSelf();
    updatePerformanceStats();
}

void Server::stop() {
    deviceObserver_->stop();
    fliquer_->stop();
    managementInterface_->stop();
    performanceStatsUpdateTimer_.cancel();

    for (auto &conn : activeDevices_) {
        conn->stop();
    }
}

void Server::registerDevice(const std::string &path,
                            std::shared_ptr<SerialChannel> conn,
                            const std::string &serial, uint16_t versionMajor,
                            uint8_t versionMinor) {
    BOOST_LOG_TRIVIAL(info) << path << ": Established connection, version "
                            << +versionMajor << '.' << +versionMinor;
    const auto announce = [&](const ChannelServer &chan,
                              std::string serialExt = "") {
        auto fullSerial = serial + serialExt;

        fliquer::Resource r;
        r.type = "tiqi.devil.channel";
        r.id = fullSerial;
        r.port = chan.rpcPort();

        fliquer::SemVer v;
        v.major = versionMajor;
        v.minor = versionMinor;
        r.version = v;

        // See if the user has configured a name string for this device.
        // Otherwise, just reuse the serial string as the display name.
        const auto nameIt = config_.channelNames.find(fullSerial);
        if (nameIt == config_.channelNames.end()) {
            r.displayName = fullSerial;
        } else {
            r.displayName = nameIt->second;
        }

        fliquer_->addLocalResource(r);

        auto self = shared_from_this();
        conn->addShutdownCallback(
            [this, self, r] { fliquer_->removeLocalResource(r); });
    };

    if (versionMajor == 1 || versionMajor == 3) {
        auto chan = ChannelServer::make(ioService_, conn);
        chan->start();
        announce(*chan);
    } else if (versionMajor == 2) {
        auto a = std::make_shared<DualChannelAdapter>(
            conn, DualChannelAdapter::Subchannel::a);
        auto chanA = ChannelServer::make(ioService_, a);
        chanA->start();
        announce(*chanA, ":A");

        auto b = std::make_shared<DualChannelAdapter>(
            conn, DualChannelAdapter::Subchannel::b);
        auto chanB = ChannelServer::make(ioService_, b);
        chanB->start();
        announce(*chanB, ":B");
    } else {
        BOOST_LOG_TRIVIAL(warning)
            << path << ": Do not know how to handle version " << +versionMajor
            << "." << +versionMinor;
    }
}

void Server::announceSelf() {
    fliquer::Resource r;
    r.type = "tiqi.devil.server";
    r.id = config_.id;
    r.displayName = config_.displayName;
    r.port = managementInterface_->port();

    r.version.major = 1;
    r.version.minor = 2;
    r.version.buildMetadata = buildTimestamp();

    fliquer_->addLocalResource(r);
}

void Server::updatePerformanceStats() {
    performanceStatsUpdateTimer_.expires_from_now(statUpdateInterval);
    performanceStatsUpdateTimer_.async_wait([&](const error_code &ec) {
        if (ec == errc::operation_canceled) return;

        const auto toHz = 1.0s / statUpdateInterval;
        performanceStats_["Serial Command Rate / Hz"] =
            performanceCounters_->serialCommandsSent * toHz;
        performanceStats_["Serial Receive Rate / Mb/s"] =
            performanceCounters_->serialBytesReceived * 8 / 1e6 * toHz;
        performanceCounters_->reset();

        updatePerformanceStats();
    });
}
}

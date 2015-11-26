#ifndef DEVIL_SERVER_HPP
#define DEVIL_SERVER_HPP

#include <memory>
#include <unordered_map>
#include "boost/asio/io_service.hpp"
#include "boost/asio/steady_timer.hpp"
#include "devil/DeviceObserver.hpp"
#include "devil/ChannelServer.hpp"
#include "devil/PerformanceCounters.hpp"
#include "devil/SerialChannel.hpp"
#include "fliquer/fliquer.hpp"

namespace devil {

/// \brief The main server instance that exists for the whole application
/// lifetime.
///
/// Reacts to connected/disconnected hardware devices by adding/removing
/// ChannelServers and exposes a simple management msgpack-rpc/ZeroMQ interface
/// (currently only used for the gathered performance statistics).
class Server : public std::enable_shared_from_this<Server> {
public:
    /// \brief Server configuration.
    struct Config {
        /// \brief Server id, used for the Fliquer resource of the server itself
        /// and as part of the ids of channels without hardware serial string.
        std::string id;

        /// \brief Display name for the server statistics Fliquer resource.
        std::string displayName;

        /// \brief Map from hardware resource ids to channel names.
        std::unordered_map<std::string, std::string> channelNames;

        /// \brief The minimum hardware streaming interval register for packets
        /// longer than 512 samples (to avoid serial communication glitches).
        RegValue minLongStreamIntervalReg;
    };

    using ChannelNameMap = std::unordered_map<std::string, std::string>;
    /// \brief Constructs a new instance.
    ///
    /// The constructor itself is not public as the chosen strategy to deal with
    /// memory management in the face of the asynchronous callbacks relies on
    /// the lifetime being managed by std::shared_ptr.
    static std::shared_ptr<Server> make(boost::asio::io_service &ioService,
                                        Config config) {
        return std::shared_ptr<Server>(
            new Server(ioService, std::move(config)));
    }

    /// \brief Starts asynchronously listening for device events.
    ///
    /// Invalid to call more than once, even after #stop() has been invoked.
    void start();

    /// \brief Signals the implementation to exit cleanly at the next possible
    /// point in time.
    ///
    /// Should not leave any active async operations behind so that boost::asio
    /// can be shut down cleanly afterwards.
    void stop();

private:
    Server(boost::asio::io_service &ioService, Config config);

    /// Spins up the ChannelServer(s) for the given connection (which is
    /// expected to already be initialized) and announces them on the Fliquer
    /// network.
    void registerDevice(const std::string &path,
                        std::shared_ptr<SerialChannel> conn,
                        const std::string &serial, uint16_t versionMajor,
                        uint8_t versionMinor);

    /// Announces the management interface on the Fliquer network.
    void announceSelf();

    /// Asynchronously waits for the performance stats update timer to expire,
    /// updates the aggregated stats and restarts the timer.
    void updatePerformanceStats();

    boost::asio::io_service &ioService_;
    const Config config_;

    std::shared_ptr<DeviceObserver> deviceObserver_;
    std::shared_ptr<fliquer::Node> fliquer_;

    std::vector<std::shared_ptr<SerialChannel>> activeDevices_;

    std::shared_ptr<PerformanceCounters> performanceCounters_;

    /// The string -> rate in double version of the performance counters, as
    /// exposed by the msgpack-rpc interface.
    std::unordered_map<std::string, double> performanceStats_;

    /// Timer expiring every statUpdateInterval seconds.
    boost::asio::steady_timer performanceStatsUpdateTimer_;

    std::shared_ptr<RpcInterface> managementInterface_;
};
}

#endif

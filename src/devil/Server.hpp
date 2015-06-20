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

class Server : public std::enable_shared_from_this<Server> {
public:
    struct Config {
        std::string id;
        std::string displayName;
        std::unordered_map<std::string, std::string> channelNames;
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

    void start();

    void stop();

private:
    Server(boost::asio::io_service &ioService, Config config);

    void registerDevice(const std::string &path,
                        std::shared_ptr<SerialChannel> conn,
                        const std::string &serial, uint16_t versionMajor,
                        uint8_t versionMinor);

    void announceSelf();

    void updatePerformanceStats();

    boost::asio::io_service &ioService_;
    const Config config_;

    std::shared_ptr<DeviceObserver> deviceObserver_;
    std::shared_ptr<fliquer::Node> fliquer_;

    std::vector<std::shared_ptr<SerialChannel>> activeDevices_;

    std::shared_ptr<PerformanceCounters> performanceCounters_;
    std::unordered_map<std::string, double> performanceStats_;
    boost::asio::steady_timer performanceStatsUpdateTimer_;
    std::shared_ptr<RpcInterface> managementInterface_;
};
}

#endif

#ifndef EVIL_SERVER_HPP
#define EVIL_SERVER_HPP

#include <memory>
#include <unordered_map>
#include "boost/asio/io_service.hpp"
#include "boost/asio/steady_timer.hpp"
#include "evil/DeviceObserver.hpp"
#include "evil/NetworkChannel.hpp"
#include "evil/PerformanceCounters.hpp"
#include "evil/SerialConnection.hpp"
#include "fliquer/fliquer.hpp"

namespace evil {

class Server : public std::enable_shared_from_this<Server> {
public:
    using ChannelNameMap = std::unordered_map<std::string, std::string>;
    static std::shared_ptr<Server> make(boost::asio::io_service &ioService,
                                        std::string serverId,
                                        ChannelNameMap channelNames) {
        return std::shared_ptr<Server>(new Server(
            ioService, std::move(serverId), std::move(channelNames)));
    }

    void start();

    void stop();

private:
    Server(boost::asio::io_service &ioService, std::string serverId,
           ChannelNameMap channelNames);

    void registerDevice(const std::string &path,
                        std::shared_ptr<SerialConnection> conn,
                        const std::string &serial, uint16_t versionMajor,
                        uint8_t versionMinor);

    void announceSelf();

    void updatePerformanceStats();

    boost::asio::io_service &ioService_;
    const std::string serverId_;
    ChannelNameMap channelNames_;

    std::shared_ptr<DeviceObserver> deviceObserver_;
    std::shared_ptr<fliquer::Node> fliquer_;

    std::vector<std::shared_ptr<SerialConnection>> activeDevices_;

    std::shared_ptr<PerformanceCounters> performanceCounters_;
    std::unordered_map<std::string, double> performanceStats_;
    boost::asio::steady_timer performanceStatsUpdateTimer_;
    std::shared_ptr<RpcInterface> managementInterface_;
};
}

#endif

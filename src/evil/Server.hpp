#ifndef EVIL_SERVER_HPP
#define EVIL_SERVER_HPP

#include <memory>
#include <unordered_map>
#include "boost/asio/io_service.hpp"
#include "evil/DeviceObserver.hpp"
#include "evil/NetworkChannel.hpp"
#include "evil/SerialConnection.hpp"
#include "fliquer/fliquer.hpp"

namespace evil {

class Server : public std::enable_shared_from_this<Server> {
public:
    using ChannelNameMap = std::unordered_map<std::string, std::string>;
    static std::shared_ptr<Server> make(boost::asio::io_service &ioService,
                                        ChannelNameMap channelNames) {
        return std::shared_ptr<Server>(
            new Server(ioService, std::move(channelNames)));
    }

    void start();

    void stop();

private:
    Server(boost::asio::io_service &ioService, ChannelNameMap channelNames);

    void registerDevice(const std::string &path,
                        std::shared_ptr<SerialConnection> conn,
                        const std::string &serial, uint16_t versionMajor,
                        uint8_t versionMinor);

    boost::asio::io_service &ioService_;
    ChannelNameMap channelNames_;

    std::shared_ptr<DeviceObserver> deviceObserver_;
    std::shared_ptr<fliquer::Node> fliquer_;

    std::vector<std::shared_ptr<SerialConnection>> activeDevices_;
};
}

#endif

#ifndef EVIL_SERVER_HPP
#define EVIL_SERVER_HPP

#include <memory>
#include <unordered_map>
#include "boost/asio/io_service.hpp"
#include "evil/DeviceObserver.hpp"
#include "fliquer/fliquer.hpp"

namespace evil {

class Server : public std::enable_shared_from_this<Server> {
public:
    using ChannelNameMap = std::unordered_map<std::string, std::string>;
    Server(boost::asio::io_service &ioService, ChannelNameMap channelNames);

    void start();

    void stop();

private:
    ChannelNameMap channelNames_;

    std::shared_ptr<DeviceObserver> deviceObserver_;
    std::shared_ptr<fliquer::Node> fliquer_;
};
}

#endif

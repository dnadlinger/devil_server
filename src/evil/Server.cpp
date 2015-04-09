#include "Server.hpp"

using namespace boost::asio;
using namespace boost::system;

namespace evil {

Server::Server(io_service &ioService, ChannelNameMap channelNames)
    : channelNames_{channelNames},
      deviceObserver_{std::make_shared<DeviceObserver>(
          ioService, [this](std::string path, std::string serial) {},
          [this](std::string path, std::string serial) {})},
      fliquer_{std::make_shared<fliquer::Node>(ioService)} {}

void Server::start() {
    deviceObserver_->start();
    fliquer_->start();
}

void Server::stop() {
    deviceObserver_->stop();
    fliquer_->stop();
}
}

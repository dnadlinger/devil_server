#ifndef EVIL_ZMQSOCKET_HPP
#define EVIL_ZMQSOCKET_HPP

#include <cstdint>
#include "azmq/socket.hpp"
#include "boost/asio/io_service.hpp"

namespace evil {

struct ZmqSocket {
    ZmqSocket(boost::asio::io_service &ioService, int type);

    void close();

    azmq::socket socket;
    uint16_t port;
};
}

#endif

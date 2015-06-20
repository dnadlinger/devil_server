#ifndef DEVIL_ZMQSOCKET_HPP
#define DEVIL_ZMQSOCKET_HPP

#include <cstdint>
#include "azmq/socket.hpp"
#include "boost/asio/io_service.hpp"

namespace devil {

/// \brief Keeps a azmq::socket object together with some metadata and
/// convenience functions.
///
/// Not an abstraction to be proud of.
struct ZmqSocket {
    ZmqSocket(boost::asio::io_service &ioService, int type);

    void close();

    azmq::socket socket;
    uint16_t port;
};
}

#endif

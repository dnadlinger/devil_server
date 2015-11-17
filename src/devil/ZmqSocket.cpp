#include "devil/ZmqSocket.hpp"

using namespace boost::asio;

namespace devil {

ZmqSocket::ZmqSocket(io_service &ioService, int type)
    : socket{ioService, type, true} {
    // Listen on any device, using a system-assigned port.
    socket.bind("tcp://*:*");

    // Parse the endpoint string to get the chosen port.
    const auto e = socket.endpoint();
    port = std::stoi(e.substr(e.find_last_of(':') + 1));
}

void ZmqSocket::close() {
    socket.cancel();

    // With ZeroMQ 4.1.3 and up (Git master), the actual endpoint address is
    // resolved after binding in a way that is not reflected in
    // azmq::socket::endpoint(). Thus, manually fetch the respective socket
    // option first.
    std::array<char, 256> endpointBuf;
    azmq::socket::last_endpoint endpointOpt{endpointBuf.data(),
                                            endpointBuf.size()};
    socket.get_option(endpointOpt);
    std::string endpointString{endpointBuf.data()};
    socket.unbind(endpointString);
}
}

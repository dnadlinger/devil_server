#include "evil/ZmqSocket.hpp"

using namespace boost::asio;

namespace evil {

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
    socket.unbind(socket.endpoint());
}
}

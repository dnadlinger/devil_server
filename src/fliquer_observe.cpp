/// Simple Fliquer client that logs all resources to the console as they are
/// discovered.

#include <iostream>
#include "boost/asio/io_service.hpp"
#include "fliquer/fliquer.hpp"

int main() {
    boost::asio::io_service io;

    auto node = fliquer::Node::make(
        io, [](const std::vector<fliquer::RemoteResource> &rrs) {
            for (const auto &rr : rrs) {
                const auto &addr = rr.first;
                const auto &res = rr.second;

                std::cout << " :: New resource appeared at " << addr << ": "
                          << res.displayName << " (" << res.type << ", "
                          << res.id << ")" << std::endl;
            }
        });

    node->start();

    io.run();

    return 0;
}

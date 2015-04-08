#include <iostream>
#include "boost/asio/io_service.hpp"
#include "fliquer/fliquer.hpp"

int main() {
    boost::asio::io_service io;

    auto node = std::make_shared<fliquer::Node>(
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

#include <iostream>
#include "boost/asio.hpp"
#include "fliquer/fliquer.hpp"

using namespace std::literals;

int main() {
    boost::asio::io_service io;

    auto node = std::make_shared<fliquer::Node>(
        io, [](const fliquer::RemoteResource &rr) {
            const auto &addr = rr.first;
            const auto &res = rr.second;

            std::cout << " :: New resource appeared at " << addr << ": "
                      << res.displayName << " (" << res.type << ", " << res.id
                      << ")" << std::endl;

        });

    node->enumerateRemoteResources(
        5s, [](const std::vector<fliquer::RemoteResource> &rrs) {
            std::cout << " :: Initial discovery: " << std::endl;
            if (rrs.empty()) {
                std::cout << "  :: (none)" << std::endl;
                return;
            }

            for (const auto &rr : rrs) {
                const auto &addr = rr.first;
                const auto &res = rr.second;
                std::cout << "  :: " << addr << ": " << res.displayName << " ("
                          << res.type << ", " << res.id << ")" << std::endl;
            }
        });

    node->start();

    io.run();

    return 0;
}

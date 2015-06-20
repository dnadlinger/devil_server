/// Simple script that fakes a Fliquer resource for protocol development.

#include <iostream>
#include "boost/asio/io_service.hpp"
#include "fliquer/fliquer.hpp"

int main() {
    boost::asio::io_service io;

    auto node = fliquer::Node::make(io);

    node->start();

    fliquer::Resource r;
    r.type = "fliquer.fake";
    r.id = "fake000";
    r.displayName = "Fake resource";
    r.port = 1234;
    r.version.major = 0;
    r.version.minor = 0;
    r.version.patch = 1;

    node->addLocalResource(r);

    io.run();

    return 0;
}

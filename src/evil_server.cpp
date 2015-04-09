#include <iostream>
#include <string>
#include <unordered_map>
#include "boost/asio/io_service.hpp"
#include "boost/asio/signal_set.hpp"
#include "boost/filesystem.hpp"
#include "boost/log/trivial.hpp"
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include "evil/Server.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace {

const auto configFileName = "evil_server.json";

std::unordered_map<std::string, std::string> readChannelNames() {
    using namespace boost::property_tree;

    std::unordered_map<std::string, std::string> result;

    if (!boost::filesystem::exists(configFileName)) return result;

    ptree tree;
    read_json(configFileName, tree);

    for (const auto &id : tree.get_child("channelNames")) {
        result[id.first] = id.second.data();
    }
    return result;
}
}

int main() {
    auto channelNames = readChannelNames();

    io_service io;

    auto server = std::make_shared<evil::Server>(io, channelNames);
    server->start();

    signal_set shutdownSignals{io};
    shutdownSignals.add(SIGINT);
    shutdownSignals.add(SIGTERM);
    shutdownSignals.add(SIGQUIT);
    shutdownSignals.async_wait([&](const error_code &, int) {
        BOOST_LOG_TRIVIAL(info) << "Shutting down...";
        server->stop();
    });

    io.run();

    BOOST_LOG_TRIVIAL(info) << "Shutdown completed.";
    return 0;
}


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

struct Config {
    std::string serverId;
    std::unordered_map<std::string, std::string> channelNames;
};

Config readConfig() {
    using namespace boost::property_tree;

    if (!boost::filesystem::exists(configFileName)) {
        Config c;
        c.serverId = "[unnamed]";
        return c;
    }

    ptree tree;
    read_json(configFileName, tree);

    std::string serverId = tree.get_child("serverId").data();

    std::unordered_map<std::string, std::string> channelNames;
    for (const auto &id : tree.get_child("channelNames")) {
        channelNames[id.first] = id.second.data();
    }

    return {std::move(serverId), std::move(channelNames)};
}
}

int main() {
    io_service io;

    Config config;

    try {
        config = readConfig();
    } catch (std::runtime_error &err) {
        BOOST_LOG_TRIVIAL(fatal)
            << "Malformed configuration file: " << err.what();
        return 1;
    }

    auto server = evil::Server::make(io, config.serverId, config.channelNames);
    server->start();

    signal_set shutdownSignals{io};
    shutdownSignals.add(SIGINT);
    shutdownSignals.add(SIGTERM);
    shutdownSignals.add(SIGQUIT);
    shutdownSignals.async_wait([&](const error_code &, int) {
        BOOST_LOG_TRIVIAL(info) << "Shutting down...";
        server->stop();
    });

    BOOST_LOG_TRIVIAL(info) << "Server started.";
    io.run();

    BOOST_LOG_TRIVIAL(info) << "Shutdown completed.";
    return 0;
}

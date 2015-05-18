
#include <iostream>
#include <string>
#include <unordered_map>
#include "boost/asio/io_service.hpp"
#include "boost/asio/signal_set.hpp"
#include "boost/filesystem.hpp"
#include "boost/log/trivial.hpp"
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include "devil/Server.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace {

const auto configFileName = "devild.json";

devil::Server::Config readConfig() {
    using namespace boost::property_tree;

    if (!boost::filesystem::exists(configFileName)) {
        devil::Server::Config c;
        c.id = "(noserverid)";
        c.displayName = "(no name configured)";
        return c;
    }

    ptree tree;
    read_json(configFileName, tree);

    const auto serverId = tree.get_child("serverId").data();
    const auto serverDisplayName = tree.get_child("serverDisplayName").data();

    std::unordered_map<std::string, std::string> channelNames;
    for (const auto &id : tree.get_child("channelNames")) {
        channelNames[id.first] = id.second.data();
    }

    return {std::move(serverId), std::move(serverDisplayName),
            std::move(channelNames)};
}
}

int main() {
    io_service io;

    devil::Server::Config config;

    try {
        config = readConfig();
    } catch (std::runtime_error &err) {
        BOOST_LOG_TRIVIAL(fatal)
            << "Malformed configuration file: " << err.what();
        return 1;
    }

    auto server = devil::Server::make(io, config);
    server->start();

    signal_set shutdownSignals{io};
    shutdownSignals.add(SIGINT);
    shutdownSignals.add(SIGTERM);
    shutdownSignals.add(SIGQUIT);
    shutdownSignals.async_wait([&](const error_code &, int) {
        BOOST_LOG_TRIVIAL(info) << "Shutting down...";
        server->stop();
    });

    BOOST_LOG_TRIVIAL(info) << "Server '" << config.displayName << "' started.";
    io.run();

    BOOST_LOG_TRIVIAL(info) << "Shutdown completed.";
    return 0;
}

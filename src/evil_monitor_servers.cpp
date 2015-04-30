/// Quick-and-dirty script to display performance stats for all EVIL servers on
/// local network.

#include <iostream>
#include <tuple>
#include <vector>
#include "azmq/socket.hpp"
#include "boost/asio.hpp"
#include "boost/asio/steady_timer.hpp"
#include "fliquer/fliquer.hpp"
#include "msgpack.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace fliquer;
using namespace msgpack;
using namespace std;

int main() {
    io_service io;

    using Server = pair<RemoteResource, azmq::socket>;
    vector<unique_ptr<Server>> servers;

    auto node = Node::make(io, [&](const vector<RemoteResource> &rrs) {
        for (auto &rr : rrs) {
            if (rr.second.type != "tiqi.evil.server") continue;

            servers.emplace_back(new Server(rr, azmq::socket(io, ZMQ_REQ)));
            auto &sock = servers.back()->second;
            sock.connect("tcp://" + rr.first.to_string() + ":" +
                         to_string(rr.second.port));
            sock.set_option(azmq::socket::snd_timeo(1000));
            sock.set_option(azmq::socket::rcv_timeo(1000));
        }
    });
    node->start();

    sbuffer requestBuf;
    using empty = type::tuple<>;
    pack(requestBuf, type::tuple<unsigned, unsigned, std::string, empty>(
                         0, 0, "perfStats", empty()));

    steady_timer timer{io};
    std::function<void()> displayStats = [&]() {
        cout << "\033c" << endl;
        cout << "EVIL Server Status" << endl;
        cout << "------------------" << endl;

        vector<Server *> toRemove;

        for (auto &s : servers) {
            azmq::message response;
            try {
                s->second.send(buffer(requestBuf.data(), requestBuf.size()));
                s->second.receive(response);
            } catch (system_error &err) {
                if (err.code().value() != EAGAIN) throw err;
                toRemove.push_back(s.get());
                continue;
            }

            cout << endl;
            cout << "Address:\t " << s->first.first << endl;
            cout << "Name:   \t " << s->first.second.displayName << endl;
            cout << "Version:\t " << s->first.second.version << endl;

            auto unp =
                msgpack::unpack(buffer_cast<const char *>(response.buffer()),
                                buffer_size(response.buffer()));

            using Stats = type::assoc_vector<std::string, double>;
            const auto msg =
                unp.get()
                    .as<type::tuple<unsigned, unsigned, type::nil, Stats>>();
            auto stats = msg.get<3>();
            std::sort(stats.begin(), stats.end(),
                      [&](auto a, auto b) { return a.first < b.first; });
            for (auto &p : stats) {
                cout << p.first << ":\t " << p.second << endl;
            }
        }

        for (auto &s : toRemove) {
            servers.erase(find_if(servers.begin(), servers.end(),
                                  [&](auto &a) { return a.get() == s; }));
        }

        timer.expires_from_now(1s);
        timer.async_wait([&](const error_code &) { displayStats(); });
    };
    displayStats();

    io.run();

    return 0;
}

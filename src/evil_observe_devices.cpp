#include <iostream>
#include "boost/asio.hpp"
#include "evil/DeviceObserver.hpp"

using namespace std::literals;

int main() {
    boost::asio::io_service io;

    auto observer = std::make_shared<evil::DeviceObserver>(
        io,
        [](auto path, auto serial) {
            std::cout << " :: Device plugged in: " << path
                      << " (serial: " << serial << ")" << std::endl;
        },
        [](auto path, auto serial) {
            std::cout << " :: Device removed: " << path
                      << " (serial: " << serial << ")" << std::endl;
        });

    observer->start();

    io.run();
    return 0;
}

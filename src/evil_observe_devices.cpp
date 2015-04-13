#include <iostream>
#include "boost/asio/io_service.hpp"
#include "evil/DeviceObserver.hpp"

int main() {
    boost::asio::io_service io;

    auto observer = evil::DeviceObserver::make(
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

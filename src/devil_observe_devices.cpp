/// Simple script to test the devil::DeviceObserver.
///
/// Prints a console message for all connect/disconnect events.

#include <iostream>
#include "boost/asio/io_service.hpp"
#include "devil/DeviceObserver.hpp"

int main() {
    boost::asio::io_service io;

    auto observer = devil::DeviceObserver::make(
        io, "local",
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

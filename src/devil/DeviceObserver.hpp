#ifndef DEVIL_DEVICEOBSERVER_HPP
#define DEVIL_DEVICEOBSERVER_HPP

#include <functional>
#include <memory>
#include <string>
#include "boost/asio/io_service.hpp"
#include "boost/asio/posix/stream_descriptor.hpp"
#include "boost/log/sources/logger.hpp"

struct udev;
struct udev_device;
struct udev_monitor;

namespace devil {

/// The udev properties of the device type to observe. Values are easy to
/// determine using "udevadm info".
namespace target_device {
const std::string subsystem = "tty";
const std::vector<std::pair<const char *, std::string>> properties = {
    {"ID_BUS", "usb"},
    {"ID_VENDOR_ID", "0403"},
    {"ID_MODEL_ID", "6010"},
    {"ID_USB_INTERFACE_NUM", "01"}};
}

class DeviceObserver : public std::enable_shared_from_this<DeviceObserver> {
public:
    using DeviceCallback =
        std::function<void(const std::string &, const std::string &)>;

    static std::shared_ptr<DeviceObserver>
    make(boost::asio::io_service &ioService, std::string noSerialPrefix,
         DeviceCallback addCallback, DeviceCallback removeCallback) {
        return std::shared_ptr<DeviceObserver>(new DeviceObserver(
            ioService, noSerialPrefix, addCallback, removeCallback));
    }

    ~DeviceObserver();

    DeviceObserver(const DeviceObserver &) = delete;
    DeviceObserver &operator=(const DeviceObserver &) = delete;

    void start();
    void stop();

private:
    DeviceObserver(boost::asio::io_service &ioService,
                   std::string noSerialPrefix, DeviceCallback addCallback,
                   DeviceCallback removeCallback);

    void receiveEvent();

    void handleDeviceEvent(udev_device *dev);

    void invokeIfMatch(DeviceCallback callback, udev_device *dev);

    // Prefix to use if no hardware serial number is present. Ideally identifies
    // the host machine somehow.
    const std::string noSerialPrefix_;

    DeviceCallback addCallback_;
    DeviceCallback removeCallback_;

    udev *const udev_;
    udev_monitor *const udevMonitor_;
    boost::asio::posix::stream_descriptor udevStream_;

    boost::log::sources::logger log_;
};
}

#endif

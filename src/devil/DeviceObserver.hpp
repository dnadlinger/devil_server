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
///
/// This could easily be made configurable per instance of DeviceObserver if
/// you want to use the class in other contexts.
namespace target_device {
const std::string subsystem = "tty";
const std::vector<std::pair<const char *, std::string>> properties = {
    {"ID_BUS", "usb"},
    {"ID_VENDOR_ID", "0403"},
    {"ID_MODEL_ID", "6010"},
    {"ID_USB_INTERFACE_NUM", "01"}};
}

/// \brief Monitors system hot-plug events for the (dis-)connection of a given
/// type of peripheral.
///
/// This is done in a way that avoids periodic polling.
///
/// The current implementation only supports Linux; more specifically any recent
/// distribution that uses udev.
class DeviceObserver : public std::enable_shared_from_this<DeviceObserver> {
public:
    /// \brief User callback for handling a device (dis-)connection event. The
    /// first parameter is the system device node path (e.g. `/dev/…` on Linux),
    /// the second is the device id string (serial number, if present).
    using DeviceCallback =
        std::function<void(const std::string &, const std::string &)>;

    /// \brief Constructs a new instance.
    ///
    /// \param noSerialPrefix A prefix to use for the device id string if no
    ///     serial number could be read from the connected device. The rest of
    ///     the string will be derived from the device node path.
    ///
    /// \param addCallback Callback to invoke when a new device matching the
    ///     property filters has been connected.
    ///
    /// \param removeCallback Callback to invoke when a device matching the
    ///     property filters has been disconnected.
    ///
    /// The constructor itself is not public as the chosen strategy to deal with
    /// memory management in the face of the asynchronous callbacks relies on
    /// the lifetime being managed by std::shared_ptr.
    static std::shared_ptr<DeviceObserver>
    make(boost::asio::io_service &ioService, std::string noSerialPrefix,
         DeviceCallback addCallback, DeviceCallback removeCallback) {
        return std::shared_ptr<DeviceObserver>(new DeviceObserver(
            ioService, std::move(noSerialPrefix), std::move(addCallback),
            std::move(removeCallback)));
    }

    ~DeviceObserver();

    DeviceObserver(const DeviceObserver &) = delete;
    DeviceObserver &operator=(const DeviceObserver &) = delete;

    /// \brief Starts asynchronously listening for device events.
    ///
    /// Invalid to call more than once, even after #stop() has been invoked.
    void start();

    /// \brief Signals the implementation to exit cleanly at the next possible
    /// point in time.
    ///
    /// Should not leave any active async operations behind so that boost::asio
    /// can be shut down cleanly afterwards.
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

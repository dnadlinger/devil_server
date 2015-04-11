#include "DeviceObserver.hpp"

#include <libudev.h>
#include <system_error>
#include "boost/asio/socket_base.hpp"
#include "boost/log/common.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace evil {

DeviceObserver::DeviceObserver(io_service &ioService,
                               DeviceCallback addCallback,
                               DeviceCallback removeCallback)
    : addCallback_{addCallback}, removeCallback_{removeCallback},
      udev_{udev_new()}, udevMonitor_{[this]() {
          assert(udev_ && "Could not create udev context.");

          auto m = udev_monitor_new_from_netlink(udev_, "udev");
          assert(m && "Could not create monitor.");

          auto ec = udev_monitor_filter_add_match_subsystem_devtype(
              m, target_device::subsystem.c_str(), nullptr);
          assert(!ec && "Failed to add filter.");

          ec = udev_monitor_enable_receiving(m);
          assert(!ec && "Failed to enable receiving.");

          return m;
      }()},
      udevStream_{ioService, udev_monitor_get_fd(udevMonitor_)} {}

DeviceObserver::~DeviceObserver() {
    udev_monitor_unref(udevMonitor_);
    udev_unref(udev_);
}

void DeviceObserver::start() {
    if (addCallback_) {
        auto e = udev_enumerate_new(udev_);
        assert(e && "Failed to create udev enumeration context.");

        auto ec = udev_enumerate_add_match_subsystem(
            e, target_device::subsystem.c_str());
        assert(!ec && "Failed to add subsystem filter.");

        ec = udev_enumerate_scan_devices(e);
        assert(!ec && "Failed to scan devices.");

        auto firstDevEntry = udev_enumerate_get_list_entry(e);
        udev_list_entry *devEntry;
        udev_list_entry_foreach(devEntry, firstDevEntry) {
            auto path = udev_list_entry_get_name(devEntry);
            auto dev = udev_device_new_from_syspath(udev_, path);
            invokeIfMatch(addCallback_, dev);
        }

        udev_enumerate_unref(e);
    }

    receiveEvent();
}

void DeviceObserver::stop() { udevStream_.close(); }

void DeviceObserver::receiveEvent() {
    auto self = shared_from_this();
    udevStream_.async_read_some(
        null_buffers(), [this, self](const error_code &ec, size_t) {
            if (ec == errc::bad_file_descriptor) {
                // stop() has been called, exit callback chain.
                return;
            }

            auto dev = udev_monitor_receive_device(udevMonitor_);
            handleDeviceEvent(dev);
            udev_device_unref(dev);

            receiveEvent();
        });
}

void DeviceObserver::handleDeviceEvent(udev_device *dev) {
    DeviceCallback handler = nullptr;
    const auto action = udev_device_get_action(dev);
    if (!action) return;
    if (action == "add"s) {
        handler = addCallback_;
    } else if (action == "remove"s) {
        handler = removeCallback_;
    }
    if (!handler) return;

    invokeIfMatch(handler, dev);
}

void DeviceObserver::invokeIfMatch(DeviceCallback callback, udev_device *dev) {
    for (const auto &filter : target_device::properties) {
        auto p = udev_device_get_property_value(dev, filter.first);

        if (!p) p = "<null>";

        if (p != filter.second) {
            if (filter.first != "ID_BUS"s) {
                // Do not log mismatch for ID_BUS, as this leads to a lot of
                // spew due to all the fixed virtual terminals.
                BOOST_LOG(log_) << "Ignoring " << target_device::subsystem
                                << " device event as " << filter.first << " is "
                                << p << ", but target device has "
                                << filter.second;
            }
            return;
        }
    }

    auto serial = udev_device_get_property_value(dev, "ID_SERIAL_SHORT");
    if (!serial) serial = "<no serial #>";

    callback(udev_device_get_devnode(dev), serial);
}
}

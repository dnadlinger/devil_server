#include "SerialConnection.hpp"

#include <algorithm>
#include "boost/asio/read.hpp"
#include "boost/asio/write.hpp"
#include "boost/log/trivial.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace evil {

namespace {
namespace hw {

namespace commands {
const uint8_t readFromReg = 0x0 << 5;
const uint8_t writeToReg = 0x1 << 5;
const uint8_t readFromStream = 0x2 << 5;
}

namespace special_regs {
const RegIdx streamInterval = 27;
const RegIdx streamSampleCount = 28;
const RegIdx streamTriggerOffset = 29;
const RegIdx systemCondition = 30;
const RegIdx version = 31;
}

const std::array<RegIdx, 1> regsToṔoll = {{special_regs::systemCondition}};

bool isSpecialReg(RegIdx idx) {
    return idx == special_regs::streamInterval ||
           idx == special_regs::streamSampleCount ||
           idx == special_regs::streamTriggerOffset ||
           idx == special_regs::version;
}

const auto clockInterval = 1s / 96e6;
const auto minSampleClockDivider = 400;
const auto sampleClockDividerStep = 512;

RegValue sampleIntervalToReg(std::chrono::duration<double> i) {
    int reg =
        ((i / clockInterval) - minSampleClockDivider) / sampleClockDividerStep;
    return std::min(std::max(0, reg), 65535);
}

std::chrono::duration<double> sampleIntervalFromReg(RegValue r) {
    return (r * sampleClockDividerStep + minSampleClockDivider) * clockInterval;
}
}

const auto readTimeout = 0.2s;
}

SerialConnection::SerialConnection(io_service &ioService,
                                   std::string devicePath)
    : devicePath_{std::move(devicePath)}, port_{ioService, devicePath_},
      timeout_{ioService}, shuttingDown_{false} {
    port_.set_option(serial_port::baud_rate(3000000));
    port_.set_option(serial_port::character_size(8));
    port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    port_.set_option(serial_port::parity(serial_port::parity::none));
    port_.set_option(
        serial_port::flow_control(serial_port::flow_control::none));
}

void SerialConnection::start(InitializedCallback initializedCallback) {
    auto self = shared_from_this();
    spawn(port_.get_io_service(), [this, self, initializedCallback](
                                      yield_context yc) {
        // Wait a bit to give the hardware some time to start up, in case the
        // user just switched it on. The UART just stops working if we are to
        // quick (probably related to the FPGA reconfiguration). This is
        // particularly noticable when powering the Papilio board from USB for
        // testing, where hot-plugging basically does not work at all without
        // the delay. There might be some garbage written to the bus which we
        // need to ignore.
        unsigned char garbage[1];
        std::function<void()> readGarbage = [&] {
            async_read(port_, buffer(garbage),
                       [&](const error_code &, size_t bytesRead) {
                           if (bytesRead) {
                               // If we have read something, there might be
                               // more still. The actual amount of data seems
                               // to vary.
                               readGarbage();
                           }
                       });
        };
        readGarbage();
        timeout_.expires_from_now(2s);
        timeout_.async_wait(yc);

        // After the grace period has elapsed, stop the garbage read loop.
        port_.cancel();

        try {
            for (RegIdx i = 0; i < registerCount_; ++i) {
                registerCache_[i] = readRegister(i, yc);
            }

            const auto version = registerCache_[hw::special_regs::version];
            const auto versionMajor = version / 100;
            const auto versionMinor = version % 100;

            initializedCallback(versionMajor, versionMinor);

            mainLoop(yc);
        } catch (system_error &err) {
            if (err.code() != error::eof &&
                err.code() != error::bad_descriptor) {
                // If the error is not just due to the EVIL being disconnected
                // or the server being shut down gracefully, log it.
                BOOST_LOG_TRIVIAL(info)
                    << "Error in serial communication; terminating connection: "
                    << err.what();
            }
        } catch (std::runtime_error &err) {
            BOOST_LOG_TRIVIAL(warning)
                << "Unexpected error in serial communication: " << err.what();
        }

        for (auto &cb : shutdownCallbacks_) {
            cb();
        }
        shutdownCallbacks_.clear();
        registerChangeCallbacks_.clear();
        for (auto &c : streamPacketCallbacks_) {
            c = nullptr;
        }
    });
}

void SerialConnection::mainLoop(yield_context yc) {
    while (!shuttingDown_) {
        // TODO: Try to cope with the case where the FPGA got only half the
        // command by sending a bunch of 1-byte no-ops and then ignore any data
        // arriving immediately after that.
        try {
            while (!shuttingDown_) {
                while (!pendingRegisterWrites_.empty()) {
                    RegIdx idx;
                    RegValue value;

                    std::tie(idx, value) = pendingRegisterWrites_.front();
                    pendingRegisterWrites_.pop_front();

                    writeRegister(idx, value, yc);
                }

                for (auto &idx : hw::regsToṔoll) {
                    const auto newVal = readRegister(idx, yc);
                    if (registerCache_[idx] != newVal) {
                        registerCache_[idx] = newVal;
                        for (auto &cb : registerChangeCallbacks_) {
                            cb(idx, newVal);
                        }
                    }
                }

                for (StreamIdx i = 0; i < streamCount_; ++i) {
                    const auto &cb = streamPacketCallbacks_[i];
                    if (cb) {
                        cb(readStreamPacket(i, streamConfigs_[i], yc));
                    }
                }
            }
        } catch (system_error &err) {
            if (err.code() != errc::operation_canceled) throw err;

            if (shuttingDown_) return;

            BOOST_LOG_TRIVIAL(info)
                << devicePath_
                << ": Operation timed out, trying to rescue connection.";
        }
    }
}

void SerialConnection::stop() {
    shuttingDown_ = true;
    port_.cancel();
}

void SerialConnection::addShutdownCallback(ShutdownCallback cb) {
    shutdownCallbacks_.push_back(cb);
}

bool SerialConnection::isValidRegister(RegIdx idx) {
    return idx < registerCount_ && !hw::isSpecialReg(idx);
}

RegValue SerialConnection::readRegister(RegIdx idx) {
    assert(isValidRegister(idx) && "Not a valid register.");
    return registerCache_[idx];
}

bool SerialConnection::modifyRegister(RegIdx idx, RegValue oldVal,
                                      RegValue newVal) {
    // Sanity check: Client is never allowed to modify special registers…
    if (hw::isSpecialReg(idx)) return false;

    auto &cache = registerCache_[idx];
    if (cache != oldVal) return false;
    if (cache == newVal) return true;
    cache = newVal;

    for (auto &cb : registerChangeCallbacks_) {
        cb(idx, newVal);
    }

    pendingRegisterWrites_.emplace_back(idx, newVal);
    return true;
}

void SerialConnection::addRegisterChangeCallback(RegisterChangeCallback cb) {
    registerChangeCallbacks_.push_back(cb);
}

StreamIdx SerialConnection::streamCount() { return streamCount_; }

StreamAcquisitionConfig
SerialConnection::streamAcquisitionConfig(StreamIdx idx) {
    return streamConfigs_[idx];
}

void SerialConnection::setStreamAcquisitionConfig(
    StreamIdx idx, const StreamAcquisitionConfig &config) {
    streamConfigs_[idx] = config;
}

void SerialConnection::setStreamPacketCallback(StreamIdx idx,
                                               StreamPacketCallback cb) {
    streamPacketCallbacks_[idx] = cb;
}

RegValue SerialConnection::readRegister(RegIdx idx, yield_context yc) {
    std::array<uint8_t, 1> writeBuf;
    writeBuf[0] = hw::commands::readFromReg | idx;
    async_write(port_, buffer(writeBuf), yc);

    std::array<RegValue, 1> readBuf;
    armTimeout(readTimeout);
    async_read(port_, buffer(readBuf), yc);
    timeout_.cancel();

    return readBuf[0];
}

void SerialConnection::writeRegister(RegIdx idx, RegValue value,
                                     yield_context yc) {
    std::array<uint8_t, 3> writeBuf;
    writeBuf[0] = hw::commands::writeToReg | idx;
    writeBuf[1] = static_cast<uint8_t>(value);
    writeBuf[2] = static_cast<uint8_t>(value >> 8);
    async_write(port_, buffer(writeBuf), yc);
}

StreamPacket SerialConnection::readStreamPacket(
    StreamIdx idx, const StreamAcquisitionConfig &config, yield_context yc) {
    auto &countCache = registerCache_[hw::special_regs::streamSampleCount];
    if (countCache != config.sampleCount) {
        countCache = config.sampleCount;
        writeRegister(hw::special_regs::streamSampleCount, countCache, yc);
    }

    auto &intervalCache = registerCache_[hw::special_regs::streamInterval];
    const auto intervalReg =
        hw::sampleIntervalToReg(config.timeSpan / config.sampleCount);
    if (intervalCache != intervalReg) {
        intervalCache = intervalReg;
        writeRegister(hw::special_regs::streamInterval, intervalCache, yc);
    }

    const auto actualInterval = hw::sampleIntervalFromReg(intervalReg);

    std::array<uint8_t, 1> writeBuf;
    writeBuf[0] = hw::commands::readFromStream | idx;
    async_write(port_, buffer(writeBuf), yc);

    // Initialization value does not matter, will get overwritten anyway.
    streamBuf_.resize(config.sampleCount, 0);

    armTimeout(actualInterval * config.sampleCount + readTimeout);
    async_read(port_, buffer(streamBuf_), yc);
    timeout_.cancel();

    // The stream trigger offset register is written from the firmware-internal
    // sample counter, which counts down to zero. Since this is a bit backwards
    // to the concept of an offset/slice, convert to an index based at the first
    // sample here.
    const auto triggerOffset =
        config.sampleCount -
        readRegister(hw::special_regs::streamTriggerOffset, yc);

    return {actualInterval, triggerOffset, streamBuf_};
}

void SerialConnection::armTimeout(std::chrono::duration<double> duration) {
    timeout_.expires_from_now(
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration));

    auto self = shared_from_this();
    timeout_.async_wait([this, self](const error_code &ec) {
        if (ec != errc::operation_canceled) {
            port_.cancel();
        }
    });
}
}

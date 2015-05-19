#include "SerialConnection.hpp"

#include <algorithm>
#include "boost/asio/read.hpp"
#include "boost/asio/write.hpp"
#include "boost/log/trivial.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace devil {

namespace {
namespace hw {

namespace commands {
const uint8_t readFromReg = 0x0 << 5;
const uint8_t writeToReg = 0x1 << 5;
const uint8_t readFromStream = 0x2 << 5;

// This opcode is currently not assigned, so it acts as a no-op.
const uint8_t noop = 0x7 << 5;
}

namespace special_regs {
const RegIdx streamInterval = 27;
const RegIdx streamSampleCount = 28;
const RegIdx streamTriggerOffset = 29;
const RegIdx systemCondition = 30;
const RegIdx version = 31;
}

const std::array<RegIdx, 1> regsToPoll = {{special_regs::systemCondition}};

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
    int reg = ceil(((i / clockInterval) - minSampleClockDivider) /
                   sampleClockDividerStep);
    return std::min(std::max(0, reg), 65535);
}

std::chrono::duration<double> sampleIntervalFromReg(RegValue r) {
    return (r * sampleClockDividerStep + minSampleClockDivider) * clockInterval;
}
}

const auto readTimeout = 1.0s;
}

SerialConnection::SerialConnection(
    io_service &ioService, std::string devicePath,
    std::shared_ptr<PerformanceCounters> performanceCounters)
    : devicePath_{std::move(devicePath)}, port_{ioService, devicePath_},
      timeout_{ioService}, performanceCounters_{performanceCounters},
      shuttingDown_{false} {
    port_.set_option(serial_port::baud_rate(3000000));
    port_.set_option(serial_port::character_size(8));
    port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    port_.set_option(serial_port::parity(serial_port::parity::none));
    port_.set_option(
        serial_port::flow_control(serial_port::flow_control::none));

    nextRegPollIdx_ = 0;
    nextStreamIdx_ = 0;
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
        // the delay.

        timeout_.expires_from_now(1s);
        timeout_.async_wait(yc);

        try {
            // If the hardware has previously been abruptly disconnected without
            // resetting it afterwards or the server process was killed, it
            // might still be waiting for the second part of a command.
            // Sometimes, there also seems to be garbage written to the UART
            // when the FPGA reconfigures.
            realignProtocol(yc);

            // Initial fetch of all the registers.
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

void SerialConnection::realignProtocol(yield_context yc) {
    // Write some 1-byte no-ops to complete any partially transmitted multi-byte
    // commands. In theory, 2 bytes should be enough since our longest commands
    // are 3 bytes, but the exact amount is inconsequential.
    std::array<uint8_t, 5> noops;
    std::fill(noops.begin(), noops.end(), hw::commands::noop);
    async_write(port_, buffer(noops), yc);

    // Now, read any garbage the device might send in response. We'll keep
    // trying to read until the timeout is hit.
    armTimeout(5 * readTimeout);
    std::array<uint8_t, 1024> garbage;
    try {
        while (true) {
            async_read(port_, buffer(garbage), yc);
        }
    } catch (system_error &err) {
        // Ignore the failure due to the timeout elapsing.
        if (err.code() != errc::operation_canceled) throw err;
    }
}

void SerialConnection::mainLoop(yield_context yc) {
    while (!shuttingDown_) {
        try {
            while (!pendingRegisterWrites_.empty()) {
                RegIdx idx;
                RegValue value;

                std::tie(idx, value) = pendingRegisterWrites_.front();
                pendingRegisterWrites_.pop_front();

                writeRegister(idx, value, yc);
            }

            if (nextRegPollIdx_ < hw::regsToPoll.size()) {
                const auto idx = hw::regsToPoll[nextRegPollIdx_];

                const auto newVal = readRegister(idx, yc);
                if (registerCache_[idx] != newVal) {
                    registerCache_[idx] = newVal;
                    for (auto &cb : registerChangeCallbacks_) {
                        cb(idx, newVal);
                    }
                }

                // Before reading the next register, check if there are pending
                // register writes to keep control latency as low as possible.
                ++nextRegPollIdx_;
                continue;
            }

            if (nextStreamIdx_ < streamCount_) {
                const auto &cb = streamPacketCallbacks_[nextStreamIdx_];
                if (cb) {
                    auto packet = readStreamPacket(
                        nextStreamIdx_, streamConfigs_[nextStreamIdx_], yc);
                    if (cb) cb(packet);
                }

                // Before acquiring the next stream packet, check if there are
                // pending register writes to keep control latency as low as
                // possible.
                ++nextStreamIdx_;
                continue;
            }

            nextRegPollIdx_ = 0;
            nextStreamIdx_ = 0;
        } catch (system_error &err) {
            if (err.code() != errc::operation_canceled) throw err;

            if (shuttingDown_) return;

            BOOST_LOG_TRIVIAL(info)
                << devicePath_
                << ": Operation timed out, trying to rescue connection.";
            realignProtocol(yc);
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
    ++performanceCounters_->serialCommandsSent;

    std::array<RegValue, 1> readBuf;
    armTimeout(readTimeout);
    async_read(port_, buffer(readBuf), yc);
    performanceCounters_->serialBytesReceived += readBuf.size();

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
    ++performanceCounters_->serialCommandsSent;
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
    ++performanceCounters_->serialCommandsSent;

    // Initialization value does not matter, will get overwritten anyway.
    streamBuf_.resize(config.sampleCount, 0);

    armTimeout(actualInterval * config.sampleCount + readTimeout);
    async_read(port_, buffer(streamBuf_), yc);
    performanceCounters_->serialBytesReceived += streamBuf_.size();
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

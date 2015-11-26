#include "SerialChannel.hpp"

#include <algorithm>
#include "boost/asio/read.hpp"
#include "boost/asio/write.hpp"
#include "boost/log/trivial.hpp"

using namespace boost::asio;
using namespace boost::system;
using namespace std::literals;

namespace devil {

namespace {
/// Hardware parameters and magic numbers.
namespace hw {

/// The command opcodes from the EVIL protocol (see Verilog decoder source for
/// documentation).
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

/// The registers that are changed by the hardware side and thus need to be
/// polled actively.
const std::array<RegIdx, 1> regsToPoll = {{special_regs::systemCondition}};

/// Returns true if the given register is special and thus not accessible from
/// client code.
bool isSpecialReg(RegIdx idx) {
    return idx == special_regs::streamInterval ||
           idx == special_regs::streamSampleCount ||
           idx == special_regs::streamTriggerOffset ||
           idx == special_regs::version;
}

/// The inverse system clock frequency.
const auto clockInterval = 1s / 96e6;

/// The number of clock cycles between successive stream samples when the
/// divider register is set to 0.
const auto minSampleClockDivider = 400;

/// The increase of clock cycles between stream samples per unit increase of the
/// divider register.
const auto sampleClockDividerStep = 512;

/// Returns the sample divider register value for a given wall clock sample
/// interval. It is always rounded up, and will never exceed the hardware
/// register range.
///
/// At about 800 samples, an interval register value of 0 seems to overflow some
/// hardware buffer when using a Papilio One board connected to a Raspberry Pi 2
/// (Arch Linux, Kernel 4.1.13). As a workaround, we require at least a value
/// of 1 there. This should be fixed by increasing the base divider in the
/// bitstream.
RegValue sampleIntervalToReg(std::chrono::duration<double> timeSpan,
                             unsigned sampleCount,
                             RegValue minLongStreamIntervalReg) {
    const RegValue reg = ceil(
        ((timeSpan / sampleCount / clockInterval) - minSampleClockDivider) /
        sampleClockDividerStep);
    const RegValue minReg = (sampleCount > 512) ? minLongStreamIntervalReg : 0;
    return std::min(std::max(minReg, reg), RegValue(65535));
}

/// Converts from a sample register value to a wall clock sample interval. This
/// conversion is accurate to the precision of the used types resp. the hardware
/// clock stability.
std::chrono::duration<double> sampleIntervalFromReg(RegValue r) {
    return (r * sampleClockDividerStep + minSampleClockDivider) * clockInterval;
}
}

/// The timeout to use when waiting for serial data to arrive (the amount of
/// time it takes for the hardware samples to be acquired is taken into account
/// separately).
const auto readTimeout = 3.0s;
}

SerialChannel::SerialChannel(
    io_service &ioService, std::string devicePath,
    RegValue minLongStreamIntervalReg,
    std::shared_ptr<PerformanceCounters> performanceCounters)
    : devicePath_{std::move(devicePath)}, port_{ioService, devicePath_},
      minLongStreamIntervalReg_{minLongStreamIntervalReg}, timeout_{ioService},
      performanceCounters_{performanceCounters}, shuttingDown_{false} {
    port_.set_option(serial_port::baud_rate(3000000));
    port_.set_option(serial_port::character_size(8));
    port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    port_.set_option(serial_port::parity(serial_port::parity::none));
    port_.set_option(
        serial_port::flow_control(serial_port::flow_control::none));

    nextRegPollIdx_ = 0;
    nextStreamIdx_ = 0;
}

void SerialChannel::start(InitializedCallback initializedCallback) {
    auto self = shared_from_this();
    spawn(port_.get_io_service(), [this, self,
                                   initializedCallback](yield_context yc) {
        // Wait a bit to give the hardware some time to start up, in case the
        // user just switched it on. The UART just stops working if we are to
        // quick (probably related to the FPGA reconfiguration). This is
        // particularly noticeable when powering the Papilio board from USB for
        // testing, where hot-plugging basically does not work at all without
        // the delay.

        timeout_.expires_from_now(3s);
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

void SerialChannel::realignProtocol(yield_context yc) {
    // Write some 1-byte no-ops to complete any partially transmitted multi-byte
    // commands. In theory, 2 bytes should be enough since our longest commands
    // are 3 bytes, but the exact amount is inconsequential.
    std::array<uint8_t, 5> noops;
    std::fill(noops.begin(), noops.end(), hw::commands::noop);
    async_write(port_, buffer(noops), yc);

    // Now, read any garbage the device might send in response. We'll keep
    // trying to read until the timeout is hit.
    armTimeout(2 * readTimeout);
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

void SerialChannel::mainLoop(yield_context yc) {
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
                    // Save away the current acquisition config to operate from
                    // the same one for the whole operation even if it is
                    // changed by the user concurrently (i.e. while we wait for
                    // the data to arrive).
                    StreamAcquisitionConfig config =
                        streamConfigs_[nextStreamIdx_];
                    auto streamResult =
                        readStreamPacket(nextStreamIdx_, config, yc);
                    if (!streamResult.first) {
                        BOOST_LOG_TRIVIAL(info)
                            << devicePath_
                            << ": Received corrupt stream data, trying "
                               "to rescue connection.";
                        realignProtocol(yc);
                        continue;
                    }

                    // Need to re-check cb because it might have been reset
                    // in the meantime if we yielded execution waiting for the
                    // data to arrive.
                    if (cb) cb(streamResult.second);
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

void SerialChannel::stop() {
    shuttingDown_ = true;
    port_.cancel();
}

void SerialChannel::addShutdownCallback(ShutdownCallback cb) {
    shutdownCallbacks_.push_back(cb);
}

bool SerialChannel::isValidRegister(RegIdx idx) {
    return idx < registerCount_ && !hw::isSpecialReg(idx);
}

RegValue SerialChannel::readRegister(RegIdx idx) {
    assert(isValidRegister(idx) && "Not a valid register.");
    return registerCache_[idx];
}

bool SerialChannel::modifyRegister(RegIdx idx, RegValue oldVal,
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

void SerialChannel::addRegisterChangeCallback(RegisterChangeCallback cb) {
    registerChangeCallbacks_.push_back(cb);
}

StreamIdx SerialChannel::streamCount() { return streamCount_; }

StreamAcquisitionConfig SerialChannel::streamAcquisitionConfig(StreamIdx idx) {
    return streamConfigs_[idx];
}

void SerialChannel::setStreamAcquisitionConfig(
    StreamIdx idx, const StreamAcquisitionConfig &config) {
    streamConfigs_[idx] = config;
}

void SerialChannel::setStreamPacketCallback(StreamIdx idx,
                                            StreamPacketCallback cb) {
    streamPacketCallbacks_[idx] = cb;
}

RegValue SerialChannel::readRegister(RegIdx idx, yield_context yc) {
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

void SerialChannel::writeRegister(RegIdx idx, RegValue value,
                                  yield_context yc) {
    std::array<uint8_t, 3> writeBuf;
    writeBuf[0] = hw::commands::writeToReg | idx;
    writeBuf[1] = static_cast<uint8_t>(value);
    writeBuf[2] = static_cast<uint8_t>(value >> 8);
    async_write(port_, buffer(writeBuf), yc);
    ++performanceCounters_->serialCommandsSent;
}

std::pair<bool, StreamPacket> SerialChannel::readStreamPacket(
    StreamIdx idx, const StreamAcquisitionConfig &config, yield_context yc) {

    auto &countCache = registerCache_[hw::special_regs::streamSampleCount];
    if (countCache != config.sampleCount) {
        countCache = config.sampleCount;
        writeRegister(hw::special_regs::streamSampleCount, countCache, yc);
    }

    auto &intervalCache = registerCache_[hw::special_regs::streamInterval];
    const auto intervalReg = hw::sampleIntervalToReg(
        config.timeSpan, config.sampleCount, minLongStreamIntervalReg_);
    if (intervalCache != intervalReg) {
        intervalCache = intervalReg;
        writeRegister(hw::special_regs::streamInterval, intervalCache, yc);
    }

    StreamPacket result{hw::sampleIntervalFromReg(intervalReg), 0, streamBuf_};

    std::array<uint8_t, 1> writeBuf;
    writeBuf[0] = hw::commands::readFromStream | idx;
    async_write(port_, buffer(writeBuf), yc);
    ++performanceCounters_->serialCommandsSent;

    // Initialization value does not matter, will get overwritten anyway.
    streamBuf_.resize(config.sampleCount, 0);

    armTimeout(result.sampleInterval * config.sampleCount + readTimeout);
    async_read(port_, buffer(streamBuf_), yc);
    performanceCounters_->serialBytesReceived += streamBuf_.size();
    timeout_.cancel();

    // The stream trigger offset register is written from the firmware-internal
    // sample counter, which counts down to zero. Since this is a bit backwards
    // to the concept of an offset/slice, convert to an index based at the first
    // sample here.
    const auto triggerReg =
        readRegister(hw::special_regs::streamTriggerOffset, yc);
    result.triggerOffset = config.sampleCount - triggerReg;

    // If the trigger register value is larger than the amount of samples
    // we took, there was definitely a communication glitch.
    const auto valid = (triggerReg <= config.sampleCount);
    return {valid, result};
}

void SerialChannel::armTimeout(std::chrono::duration<double> duration) {
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

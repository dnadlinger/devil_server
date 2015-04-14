#include "SerialConnection.hpp"

#include <chrono>
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

const auto clockInterval = 1s / 96e3;
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
}

SerialConnection::SerialConnection(io_service &ioService,
                                   const std::string &devicePath)
    : port_{ioService, devicePath} {
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
        try {
            for (RegIdx i = 0; i < registerCount_; ++i) {
                registerCache_[i] = readRegister(i, yc);
            }

            const auto version = registerCache_[hw::special_regs::version];
            const auto versionMajor = version / 100;
            const auto versionMinor = version % 100;

            initializedCallback(versionMajor, versionMinor);

            while (true) {
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
                        cb(readStreamPacket(i, streamParams_[i], yc));
                    }
                }
            }
        } catch (system_error &err) {
            if (err.code() != error::eof &&
                err.code() != error::bad_descriptor &&
                err.code() != errc::operation_canceled) {
                // If the error is not just due to the EVIl being disconnected
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

void SerialConnection::stop() { port_.cancel(); }

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

void SerialConnection::configureStream(StreamIdx idx,
                                       const StreamParams &params) {
    streamParams_[idx] = params;
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
    async_read(port_, buffer(readBuf), yc);

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

StreamPacket SerialConnection::readStreamPacket(StreamIdx idx,
                                                const StreamParams &params,
                                                yield_context yc) {
    auto &countCache = registerCache_[hw::special_regs::streamSampleCount];
    if (countCache != params.sampleCount) {
        countCache = params.sampleCount;
        writeRegister(hw::special_regs::streamSampleCount, countCache, yc);
    }

    auto &intervalCache = registerCache_[hw::special_regs::streamSampleCount];
    const auto interval =
        hw::sampleIntervalToReg(params.timeSpan / params.sampleCount);
    if (intervalCache != interval) {
        intervalCache = interval;
        writeRegister(hw::special_regs::streamInterval, intervalCache, yc);
    }

    std::array<uint8_t, 1> writeBuf;
    writeBuf[0] = hw::commands::readFromStream | idx;
    async_write(port_, buffer(writeBuf), yc);

    // Initialization value does not matter, will get overwritten anyway.
    streamBuf_.resize(params.sampleCount, 0);
    async_read(port_, buffer(streamBuf_), yc);

    const auto triggerOffset =
        readRegister(hw::special_regs::streamTriggerOffset, yc);

    return {hw::sampleIntervalFromReg(interval), triggerOffset, streamBuf_};
}
}

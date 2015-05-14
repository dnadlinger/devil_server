#ifndef DEVIL_HARDWARECHANNEL_HPP
#define DEVIL_HARDWARECHANNEL_HPP

#include <chrono>
#include <functional>
#include <vector>

namespace devil {

using RegIdx = unsigned;
using RegValue = uint16_t;
using StreamIdx = unsigned;
using StreamSample = uint8_t;

struct StreamAcquisitionConfig {
    StreamAcquisitionConfig()
        : timeSpan{std::chrono::milliseconds(10)}, sampleCount(512) {}
    StreamAcquisitionConfig(std::chrono::duration<double> timeSpan,
                            unsigned sampleCount)
        : timeSpan{timeSpan}, sampleCount{sampleCount} {}

    std::chrono::duration<double> timeSpan;
    unsigned sampleCount;
};

struct StreamPacket {
    std::chrono::duration<double> sampleInterval;
    unsigned triggerOffset;
    const std::vector<StreamSample> &samples;
};

class HardwareChannel {
public:
    virtual void stop() = 0;

    using ShutdownCallback = std::function<void()>;
    virtual void addShutdownCallback(ShutdownCallback cb) = 0;

    virtual bool isValidRegister(RegIdx idx) = 0;

    virtual RegValue readRegister(RegIdx idx) = 0;

    virtual bool modifyRegister(RegIdx idx, RegValue oldVal,
                                RegValue newVal) = 0;

    using RegisterChangeCallback = std::function<void(RegIdx, RegValue)>;
    virtual void addRegisterChangeCallback(RegisterChangeCallback cb) = 0;

    virtual StreamIdx streamCount() = 0;

    /// \brief Returns the configuration currently used for acquiring stream
    /// data.
    virtual StreamAcquisitionConfig streamAcquisitionConfig(StreamIdx idx) = 0;

    /// \brief Sets a configuration for acquiring streams to use from now on.
    virtual void
    setStreamAcquisitionConfig(StreamIdx idx,
                               const StreamAcquisitionConfig &params) = 0;

    using StreamPacketCallback = std::function<void(const StreamPacket &)>;
    virtual void setStreamPacketCallback(StreamIdx idx,
                                         StreamPacketCallback cb) = 0;
};
}

#endif

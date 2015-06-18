#ifndef DEVIL_CHANNEL_HPP
#define DEVIL_CHANNEL_HPP

#include <chrono>
#include <functional>
#include <vector>

namespace devil {

/// \brief An integer index denoting one of the hardware registers.
///
/// In the EVIL firmware 2.0, 32 registers are exposed. Changing this would also
/// necessitate a modification of the serial communication protocol.
using RegIdx = unsigned;

/// \brief The 16 bit value of a hardware register.
///
/// The actual interpretation (signed/unsigned, etc.) depends on the type of
/// register. For the purposes of this server software, they are just opaque
/// bits of data.
using RegValue = uint16_t;

/// \brief An integer index denoting one of the hardware stream channels.
using StreamIdx = unsigned;

/// \brief The value of a single sample in one of the hardware stream channels.
///
/// Currently, this is hard-coded to be an 8-bit integer. In case future
/// firmware versions expose e.g. the full 10 bit ADC resolution or 16 bit DAC
/// output, the code handling the streaming data will likely have to be modified
/// to carry along the type information, as the stream sample arrays are exposed
/// as (typed) binary blobs by the network interface.
using StreamSample = uint8_t;

/// \brief The configuration settings for the hardware stream packet
/// acquisition.
struct StreamAcquisitionConfig {
    StreamAcquisitionConfig()
        : timeSpan{std::chrono::milliseconds(10)}, sampleCount(512) {}
    StreamAcquisitionConfig(std::chrono::duration<double> timeSpan,
                            unsigned sampleCount)
        : timeSpan{timeSpan}, sampleCount{sampleCount} {}

    /// \brief The minimum time span over which to acquire samples.
    ///
    /// Because the hardware sample rate might be restricted to integer
    /// fractions of the system clock, the actual time span used might
    /// be larger
    std::chrono::duration<double> timeSpan;

    /// \brief The number of samples to acquire.
    ///
    /// In contrast to timeSpan, the actual number of samples acquired will
    /// always precisely match this.
    unsigned sampleCount;
};

/// \brief A packet of stream data, as received from the hardware.
struct StreamPacket {
    /// \brief The actual time interval between two samples.
    std::chrono::duration<double> sampleInterval;

    /// \brief The index of the sample where the
    unsigned triggerOffset;

    /// \brief The samples acquired from the data stream, ordered
    /// chronologically.
    const std::vector<StreamSample> &samples;
};

/// \brief Represents a hardware controller channel to be exported by the
/// server.
///
/// This is only a pure virtual interface. Implementations might include a
/// direct translation to serial interface operations or an adapter to (de-)
/// multiplex when using multiple input/output channels per FPGA.
///
/// Implementations are expected not to block execution for a significant
/// amount of time for any of the methods.
class Channel {
public:
    /// \brief Signals the implementation to cleanly shut down communication
    /// with the hardware at the next possible point in time.
    ///
    /// Should not leave any active async operations behind so that boost::asio
    /// can be shut down cleanly afterwards.
    virtual void stop() = 0;

    using ShutdownCallback = std::function<void()>;
    /// \brief Adds a callback to be executed when the connection is shut down.
    ///
    /// The callback is invoked regardless of whether there was an unexpected
    /// communication error or #stop() was called.
    virtual void addShutdownCallback(ShutdownCallback cb) = 0;

    /// \brief Returns whether there is a register associated with the given
    /// index that is valid to access from user code.
    virtual bool isValidRegister(RegIdx idx) = 0;

    /// \brief Returns the current value of the register.
    ///
    /// For registers written from our side, this is the last set value. For
    /// hardware-updated registers, this is value obtained when it was last
    /// polled.
    virtual RegValue readRegister(RegIdx idx) = 0;

    /// \brief Tries to change the value in the given register from `oldVal` to
    /// `newVal`.
    ///
    /// This is similar in concept to a compare-and-swap in multi-threaded
    /// programming, as it allows multiple clients to coordinate in a race-free
    /// way without requiring extra synchronization. Here, the scenario does not
    /// involve multiple threads, though, but multiple network/... clients.
    ///
    /// \returns `true` if the value could be set successfully, `false` if there
    /// was a conflict.
    virtual bool modifyRegister(RegIdx idx, RegValue oldVal,
                                RegValue newVal) = 0;

    using RegisterChangeCallback = std::function<void(RegIdx, RegValue)>;
    /// \brief Adds a callback to be invoked when a register value has been
    /// changed.
    virtual void addRegisterChangeCallback(RegisterChangeCallback cb) = 0;

    /// \brief Returns the number of streaming channels the hardware supports.
    virtual StreamIdx streamCount() = 0;

    /// \brief Returns the configuration currently used for acquiring stream
    /// data on the given channel.
    virtual StreamAcquisitionConfig streamAcquisitionConfig(StreamIdx idx) = 0;

    /// \brief Sets the acquisition parameters for the specified stream; to be
    /// used for all future packets.
    virtual void
    setStreamAcquisitionConfig(StreamIdx idx,
                               const StreamAcquisitionConfig &params) = 0;

    using StreamPacketCallback = std::function<void(const StreamPacket &)>;
    /// \brief Specifies a callback to invoke when a sample packet for the given
    /// stream has been received, or `nullptr` to ignore this channel.
    ///
    /// Clients should always specify a null callback when they no longer care
    /// about a given channel, as this allows implementations to not query the
    /// hardware for stream at all, increasing the bandwidth available to the
    /// other streams.
    virtual void setStreamPacketCallback(StreamIdx idx,
                                         StreamPacketCallback cb) = 0;
};
}

#endif

#ifndef DEVIL_SERIALCHANNEL_HPP
#define DEVIL_SERIALCHANNEL_HPP

#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/spawn.hpp"
#include "boost/asio/steady_timer.hpp"
#include "devil/Channel.hpp"
#include "devil/PerformanceCounters.hpp"

namespace devil {

/// \brief Serial connection to a hardware controller that speaks the EVIL
/// protocol.
///
/// For newer hardware, a single SerialChannel might consist of multiple logical
/// channels.
class SerialChannel : public Channel,
                      public std::enable_shared_from_this<SerialChannel> {
public:
    /// \brief Constructs a new instance.
    ///
    /// The constructor itself is not public as the chosen strategy to deal with
    /// memory management in the face of the asynchronous callbacks relies on
    /// the lifetime being managed by std::shared_ptr.
    ///
    /// \param devicePath The device path of the serial interface to use (e.g.
    ///     a device file name such as `/dev/ttyUSB1` on Linux).
    /// \param performanceCounters A performance counter object to update based
    ///     on this serial connection.
    static std::shared_ptr<SerialChannel>
    make(boost::asio::io_service &ioService, std::string devicePath,
         RegValue minLongStreamIntervalReg,
         std::shared_ptr<PerformanceCounters> performanceCounters) {
        return std::shared_ptr<SerialChannel>(new SerialChannel(
            ioService, std::move(devicePath), minLongStreamIntervalReg,
            std::move(performanceCounters)));
    }

    /// \brief Callback to invoke once the connection has been established.
    ///
    /// The first parameter is the major version reported by the hardware, the
    /// second parameter the minor version.
    using InitializedCallback = std::function<void(uint16_t, uint8_t)>;

    /// \brief Starts asynchronously listening for device events.
    ///
    /// \param initializedCallback Callback to invoke once the connection has
    ///     been established and the internal state (register values, etc.) has
    ///     been synchronized.
    ///
    /// Invalid to call more than once on a single instance, even after #stop()
    /// has been invoked.
    void start(InitializedCallback initializedCallback);

    void stop() override;

    void addShutdownCallback(ShutdownCallback cb) override;

    bool isValidRegister(RegIdx idx) override;

    RegValue readRegister(RegIdx idx) override;

    bool modifyRegister(RegIdx idx, RegValue oldVal, RegValue newVal) override;

    void addRegisterChangeCallback(RegisterChangeCallback cb) override;

    StreamIdx streamCount() override;

    StreamAcquisitionConfig streamAcquisitionConfig(StreamIdx idx) override;

    void
    setStreamAcquisitionConfig(StreamIdx idx,
                               const StreamAcquisitionConfig &config) override;

    void setStreamPacketCallback(StreamIdx idx,
                                 StreamPacketCallback cb) override;

private:
    SerialChannel(boost::asio::io_service &ioService, std::string devicePath,
                  RegValue minLongStreamIntervalReg,
                  std::shared_ptr<PerformanceCounters> performanceCounters);

    /// Tries to recover the hardware connection after it was in a weird state.
    ///
    /// Unfortunately, the hardware communication protocol does not know any
    /// framing at all. Thus, if the device was left behind in an unknown state,
    /// the hardware might still be waiting for a part of a multi-byte command
    /// or sending some streaming data.
    void realignProtocol(boost::asio::yield_context yc);

    /// Runs the main loop once the connection has been initialized.
    void mainLoop(boost::asio::yield_context yc);

    /// Reads the register with the given index from the hardware.
    RegValue readRegister(RegIdx idx, boost::asio::yield_context yc);

    /// Writes the given register value to the hardware.
    void writeRegister(RegIdx idx, RegValue value,
                       boost::asio::yield_context yc);

    /// Reads a packet from the given stream into streamBuf_.
    ///
    /// \returns `true` and the received data if successful, `false` if there
    ///     was a connection problem while waiting for the data.
    std::pair<bool, StreamPacket>
    readStreamPacket(StreamIdx idx, const StreamAcquisitionConfig &config,
                     boost::asio::yield_context yc);

    /// Arms the serial communication timeout_ with the given duration.
    void armTimeout(std::chrono::duration<double> duration);

    const std::string devicePath_;

    boost::asio::serial_port port_;

    const RegValue minLongStreamIntervalReg_;

    /// Timeout for the serial communication. Armed and canceled again with
    /// each successful operation.
    boost::asio::steady_timer timeout_;

    /// Performance counters to update.
    std::shared_ptr<PerformanceCounters> performanceCounters_;

    /// Set to true by stop() so we can distinguish between intermittent
    /// connection failures and user shutdown requests.
    bool shuttingDown_;

    /// Callbacks to invoke once we have shut down.
    std::vector<ShutdownCallback> shutdownCallbacks_;

    enum { registerCount_ = 32 };

    /// Local cache of all register values, to satisfy user requests from.
    /// Always up to date after the initialization is done, except for
    /// hardware-modified, polled registers.
    std::array<RegValue, registerCount_> registerCache_;

    /// Register change callbacks as per the Channel interface.
    std::vector<RegisterChangeCallback> registerChangeCallbacks_;

    enum { streamCount_ = 8 };
    std::array<StreamAcquisitionConfig, streamCount_> streamConfigs_;
    std::array<StreamPacketCallback, streamCount_> streamPacketCallbacks_;

    /// Register write commands as
    std::deque<std::pair<RegIdx, RegValue>> pendingRegisterWrites_;

    /// The index of the next register in hw::regsToPoll to poll in the main
    /// loop. Equal to hw::regsToPoll.size() if we are done for this iteration.
    size_t nextRegPollIdx_;

    /// The index of the next stream to query in the main loop. Equal to
    /// streamCount_ if we are done for this iteration.
    size_t nextStreamIdx_;

    /// Buffer to receive stream samples into.
    std::vector<StreamSample> streamBuf_;
};
}

#endif

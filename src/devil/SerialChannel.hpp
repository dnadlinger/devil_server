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

class SerialChannel : public Channel,
                      public std::enable_shared_from_this<SerialChannel> {
public:
    static std::shared_ptr<SerialChannel>
    make(boost::asio::io_service &ioService, std::string devicePath,
         std::shared_ptr<PerformanceCounters> performanceCounters) {
        return std::shared_ptr<SerialChannel>(new SerialChannel(
            ioService, std::move(devicePath), std::move(performanceCounters)));
    }

    using InitializedCallback = std::function<void(uint16_t, uint8_t)>;
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
                  std::shared_ptr<PerformanceCounters> performanceCounters);

    void realignProtocol(boost::asio::yield_context yc);

    void mainLoop(boost::asio::yield_context yc);

    RegValue readRegister(RegIdx idx, boost::asio::yield_context yc);
    void writeRegister(RegIdx idx, RegValue value,
                       boost::asio::yield_context yc);

    std::pair<bool, StreamPacket>
    readStreamPacket(StreamIdx idx, const StreamAcquisitionConfig &config,
                     boost::asio::yield_context yc);

    void armTimeout(std::chrono::duration<double> duration);

    std::string devicePath_;
    boost::asio::serial_port port_;
    boost::asio::steady_timer timeout_;
    std::shared_ptr<PerformanceCounters> performanceCounters_;

    bool shuttingDown_;
    std::vector<ShutdownCallback> shutdownCallbacks_;

    enum { registerCount_ = 32 };
    std::array<RegValue, registerCount_> registerCache_;
    std::vector<RegisterChangeCallback> registerChangeCallbacks_;

    enum { streamCount_ = 8 };
    std::array<StreamAcquisitionConfig, streamCount_> streamConfigs_;
    std::array<StreamPacketCallback, streamCount_> streamPacketCallbacks_;

    std::deque<std::pair<RegIdx, RegValue>> pendingRegisterWrites_;
    size_t nextRegPollIdx_;
    size_t nextStreamIdx_;
    std::vector<StreamSample> streamBuf_;
};
}

#endif

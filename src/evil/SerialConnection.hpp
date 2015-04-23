#ifndef EVIL_SERIALCONNECTION_HPP
#define EVIL_SERIALCONNECTION_HPP

#include <array>
#include <deque>
#include <functional>
#include <memory>
#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "boost/asio/spawn.hpp"
#include "evil/HardwareChannel.hpp"

namespace evil {

class SerialConnection : public HardwareChannel,
                         public std::enable_shared_from_this<SerialConnection> {
public:
    static std::shared_ptr<SerialConnection>
    make(boost::asio::io_service &ioService, const std::string &devicePath) {
        return std::shared_ptr<SerialConnection>(
            new SerialConnection(ioService, devicePath));
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

    void setStreamAcquisitionConfig(
        StreamIdx idx, const StreamAcquisitionConfig &config) override;

    void setStreamPacketCallback(StreamIdx idx,
                                 StreamPacketCallback cb) override;

private:
    SerialConnection(boost::asio::io_service &ioService,
                     const std::string &devicePath);

    RegValue readRegister(RegIdx idx, boost::asio::yield_context yc);
    void writeRegister(RegIdx idx, RegValue value,
                       boost::asio::yield_context yc);

    StreamPacket readStreamPacket(StreamIdx idx,
                                  const StreamAcquisitionConfig &config,
                                  boost::asio::yield_context yc);

    boost::asio::serial_port port_;

    std::vector<ShutdownCallback> shutdownCallbacks_;

    enum { registerCount_ = 32 };
    std::array<RegValue, registerCount_> registerCache_;
    std::vector<RegisterChangeCallback> registerChangeCallbacks_;

    enum { streamCount_ = 8 };
    std::array<StreamAcquisitionConfig, streamCount_> streamConfigs_;
    std::array<StreamPacketCallback, streamCount_> streamPacketCallbacks_;

    std::deque<std::pair<RegIdx, RegValue>> pendingRegisterWrites_;
    std::vector<StreamSample> streamBuf_;
};
}

#endif

#ifndef EVIL_EVIL2ADAPTER_HPP
#define EVIL_EVIL2ADAPTER_HPP

#include <memory>
#include "evil/HardwareChannel.hpp"

namespace evil {

class Evil2Adapter : public HardwareChannel {
public:
    enum class Channel { a = 0, b = 1 };

    Evil2Adapter(std::shared_ptr<HardwareChannel> hw, Channel chanel);

    void stop() override;

    void addShutdownCallback(ShutdownCallback cb) override;

    bool isValidRegister(RegIdx idx) override;

    RegValue readRegister(RegIdx idx) override;

    bool modifyRegister(RegIdx idx, RegValue oldVal, RegValue newVal) override;

    void addRegisterChangeCallback(RegisterChangeCallback cb) override;

    StreamIdx streamCount() override;

    void configureStream(StreamIdx idx, const StreamParams &params) override;

    void setStreamPacketCallback(StreamIdx idx,
                                 StreamPacketCallback cb) override;

private:
    std::shared_ptr<HardwareChannel> hw_;
    Channel channel_;
};
}

#endif

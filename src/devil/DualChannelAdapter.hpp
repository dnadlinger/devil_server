#ifndef DEVIL_EVIL2CHANNEL_HPP
#define DEVIL_EVIL2CHANNEL_HPP

#include <memory>
#include "devil/Channel.hpp"

namespace devil {

/// \brief Wraps a hardware connection to an EVIL 2 dual channel device to
/// present one of the two independent controller channels as a Channel object.
class DualChannelAdapter : public Channel {
public:
    ///
    enum class Subchannel {
        a = 0, ///< Channel A ("fast")
        b = 1  ///< Channel B ("slow")
    };

    DualChannelAdapter(std::shared_ptr<Channel> hw, Subchannel sub);

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
    /// The underlying hardware channel that presents the two logical controller
    /// channels.
    std::shared_ptr<Channel> hw_;

    /// The subchannel of the hardware this object exposes.
    Subchannel sub_;
};
}

#endif

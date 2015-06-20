#include "DualChannelAdapter.hpp"

namespace devil {

namespace {
/// The first regular register of subchannel A.
const RegIdx aFirstReg = 1;
/// The last regular register of subchannel A.
const RegIdx aLastReg = 11;
/// The offset of the register numbers of subchannel B compared to the related
/// registers of subchannel A.
const RegIdx bRegOffset = 11;

// The register indices for the hardware system condition/control registers and
// the indices using which to expose them in the subchannels.
const RegIdx fakeSystemControlReg = 0;
const RegIdx realSystemControlReg = 0;
const RegIdx fakeSystemConditionReg = 30;
const RegIdx realSystemConditionReg = 30;

/// Which bits of the system control register belong to subchannel A.
const RegValue systemControlMask = 0b11111;

/// The amount of bits to shift systemControlMask to get the mask for subchannel
/// B.
const unsigned systemControlBShift = 5;

/// Converts a "fake" system control register value to the corrseponding value
/// for the real hardware value, given the current value of the real register
/// (needed as the part for the other channel must remain untouched).
RegValue encodeSystemControlReg(DualChannelAdapter::Subchannel sub,
                                RegValue currentRealVal, RegValue fakeVal) {
    auto mask = systemControlMask;
    auto shift = 0;
    if (sub == DualChannelAdapter::Subchannel::b) {
        shift = systemControlBShift;
        mask <<= systemControlBShift;
    }

    auto realVal = currentRealVal;
    realVal &= ~mask;
    realVal |= (fakeVal << shift);

    return realVal;
}

/// Extracts the "fake" system control register value for the given subchannel
/// from the true hardware register value.
RegValue decodeSystemControlReg(DualChannelAdapter::Subchannel channel,
                                RegValue val) {
    if (channel == DualChannelAdapter::Subchannel::b) {
        val >>= systemControlBShift;
    }
    return val & systemControlMask;
}

/// Extracts the "fake" system condition register value for the given subchannel
/// from the true hardware register value.
RegValue decodeSystemConditionReg(DualChannelAdapter::Subchannel channel,
                                  RegValue val) {
    if (channel == DualChannelAdapter::Subchannel::b) {
        val >>= 1;
    }
    return val & 0b1;
}

enum { streamsPerSubchannel = 4 };

/// Maps the user-facing stream indices for each of the subchannels to hardware
/// stream indices.
const std::array<std::array<StreamIdx, streamsPerSubchannel>, 2> streamMap = {
    {{{0, 2, 3, 4}}, {{1, 5, 6, 7}}}};
}

DualChannelAdapter::DualChannelAdapter(std::shared_ptr<Channel> hw,
                                       DualChannelAdapter::Subchannel sub)
    : hw_{std::move(hw)}, sub_{sub} {}

void DualChannelAdapter::stop() { hw_->stop(); }

void DualChannelAdapter::addShutdownCallback(ShutdownCallback cb) {
    hw_->addShutdownCallback(cb);
}

bool DualChannelAdapter::isValidRegister(RegIdx idx) {
    return idx == fakeSystemControlReg || idx == fakeSystemConditionReg ||
           (aFirstReg <= idx && idx <= aLastReg);
}

RegValue DualChannelAdapter::readRegister(RegIdx idx) {
    if (idx == fakeSystemConditionReg) {
        const auto val = hw_->readRegister(realSystemConditionReg);
        return decodeSystemConditionReg(sub_, val);
    }

    if (idx == fakeSystemControlReg) {
        const auto val = hw_->readRegister(realSystemControlReg);
        return decodeSystemControlReg(sub_, val);
    }

    if (sub_ == Subchannel::b) {
        idx += bRegOffset;
    }
    return hw_->readRegister(idx);
}

bool DualChannelAdapter::modifyRegister(RegIdx idx, RegValue oldVal,
                                        RegValue newVal) {
    if (idx == fakeSystemConditionReg) return false;

    if (idx == fakeSystemControlReg) {
        const auto realReg = hw_->readRegister(realSystemControlReg);
        const auto oldReg = encodeSystemControlReg(sub_, realReg, oldVal);
        const auto newReg = encodeSystemControlReg(sub_, realReg, newVal);
        return hw_->modifyRegister(realSystemControlReg, oldReg, newReg);
    }

    if (sub_ == Subchannel::b) {
        idx += bRegOffset;
    }
    return hw_->modifyRegister(idx, oldVal, newVal);
}

void DualChannelAdapter::addRegisterChangeCallback(RegisterChangeCallback cb) {
    hw_->addRegisterChangeCallback([ c = sub_, cb ](RegIdx idx, RegValue val) {
        if (idx == realSystemConditionReg) {
            cb(fakeSystemConditionReg, decodeSystemConditionReg(c, val));
            return;
        }

        if (idx == realSystemControlReg) {
            cb(fakeSystemControlReg, decodeSystemControlReg(c, val));
            return;
        }

        // Not a special register; only forward if the register actually belongs
        // to our subchannel.
        if (c == Subchannel::a && idx <= bRegOffset) {
            cb(idx, val);
        } else if (c == Subchannel::b && idx > bRegOffset) {
            cb(idx - bRegOffset, val);
        }
    });
}

StreamIdx DualChannelAdapter::streamCount() { return streamsPerSubchannel; }

StreamAcquisitionConfig
DualChannelAdapter::streamAcquisitionConfig(StreamIdx idx) {
    return hw_->streamAcquisitionConfig(
        streamMap[static_cast<unsigned>(sub_)][idx]);
}

void DualChannelAdapter::setStreamAcquisitionConfig(
    StreamIdx idx, const StreamAcquisitionConfig &config) {
    hw_->setStreamAcquisitionConfig(streamMap[static_cast<unsigned>(sub_)][idx],
                                    config);
}

void DualChannelAdapter::setStreamPacketCallback(StreamIdx idx,
                                                 StreamPacketCallback cb) {
    hw_->setStreamPacketCallback(streamMap[static_cast<unsigned>(sub_)][idx],
                                 cb);
}
}

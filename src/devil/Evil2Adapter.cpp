#include "Evil2Adapter.hpp"

namespace devil {

namespace {
const RegIdx channelBRegOffset = 11;

const RegIdx fakeSystemControlReg = 0;
const RegIdx realSystemControlReg = 0;
const RegIdx fakeSystemConditionReg = 30;
const RegIdx realSystemConditionReg = 30;

const RegValue systemControlMask = 0b11111;
const unsigned systemControlBShift = 5;

/// Converts a "fake" system control register value to the corrseponding value
/// for the real hardware value, given the current value of the real register
/// (needed as the part for the other channel must remain untouched).
RegValue encodeSystemControlReg(Evil2Adapter::Channel channel,
                                RegValue currentRealVal, RegValue fakeVal) {
    auto mask = systemControlMask;
    auto shift = 0;
    if (channel == Evil2Adapter::Channel::b) {
        shift = systemControlBShift;
        mask <<= systemControlBShift;
    }

    auto realVal = currentRealVal;
    realVal &= ~mask;
    realVal |= (fakeVal << shift);

    return realVal;
}

/// Extracts the "fake" system control register value for the given channel from
/// the true hardware register value.
RegValue decodeSystemControlReg(Evil2Adapter::Channel channel, RegValue val) {
    if (channel == Evil2Adapter::Channel::b) {
        val >>= systemControlBShift;
    }
    return val & systemControlMask;
}

/// Extracts the "fake" system condition register value for the given channel
/// from the true hardware register value.
RegValue decodeSystemConditionReg(Evil2Adapter::Channel channel, RegValue val) {
    if (channel == Evil2Adapter::Channel::b) {
        val >>= 1;
    }
    return val & 0b1;
}

const std::array<std::array<StreamIdx, 4>, 2> streamMap = {
    {{{0, 2, 3, 4}}, {{1, 5, 6, 7}}}};
}

Evil2Adapter::Evil2Adapter(std::shared_ptr<HardwareChannel> hw,
                           Evil2Adapter::Channel channel)
    : hw_{std::move(hw)}, channel_{channel} {}

void Evil2Adapter::stop() { hw_->stop(); }

void Evil2Adapter::addShutdownCallback(ShutdownCallback cb) {
    hw_->addShutdownCallback(cb);
}

bool Evil2Adapter::isValidRegister(RegIdx idx) {
    return idx == fakeSystemControlReg || idx == fakeSystemConditionReg ||
           (1 <= idx && idx <= 11);
}

RegValue Evil2Adapter::readRegister(RegIdx idx) {
    if (idx == fakeSystemConditionReg) {
        const auto val = hw_->readRegister(realSystemConditionReg);
        return decodeSystemConditionReg(channel_, val);
    }

    if (idx == fakeSystemControlReg) {
        const auto val = hw_->readRegister(realSystemControlReg);
        return decodeSystemControlReg(channel_, val);
    }

    if (channel_ == Channel::b) {
        idx += channelBRegOffset;
    }
    return hw_->readRegister(idx);
}

bool Evil2Adapter::modifyRegister(RegIdx idx, RegValue oldVal,
                                  RegValue newVal) {
    if (idx == fakeSystemConditionReg) return false;

    if (idx == fakeSystemControlReg) {
        const auto realReg = hw_->readRegister(realSystemControlReg);
        const auto oldReg = encodeSystemControlReg(channel_, realReg, oldVal);
        const auto newReg = encodeSystemControlReg(channel_, realReg, newVal);
        return hw_->modifyRegister(realSystemControlReg, oldReg, newReg);
    }

    if (channel_ == Channel::b) {
        idx += channelBRegOffset;
    }
    return hw_->modifyRegister(idx, oldVal, newVal);
}

void Evil2Adapter::addRegisterChangeCallback(RegisterChangeCallback cb) {
    hw_->addRegisterChangeCallback([ c = channel_, cb ](RegIdx idx,
                                                        RegValue val) {
        if (idx == realSystemConditionReg) {
            cb(fakeSystemConditionReg, decodeSystemConditionReg(c, val));
            return;
        }

        if (idx == realSystemControlReg) {
            cb(fakeSystemControlReg, decodeSystemControlReg(c, val));
            return;
        }

        if (c == Channel::a && idx <= channelBRegOffset) {
            cb(idx, val);
        } else if (c == Channel::b && idx > channelBRegOffset) {
            cb(idx - channelBRegOffset, val);
        }
    });
}

StreamIdx Evil2Adapter::streamCount() { return 4; }

StreamAcquisitionConfig Evil2Adapter::streamAcquisitionConfig(StreamIdx idx) {
    return hw_->streamAcquisitionConfig(
        streamMap[static_cast<unsigned>(channel_)][idx]);
}

void Evil2Adapter::setStreamAcquisitionConfig(
    StreamIdx idx, const StreamAcquisitionConfig &config) {
    hw_->setStreamAcquisitionConfig(
        streamMap[static_cast<unsigned>(channel_)][idx], config);
}

void Evil2Adapter::setStreamPacketCallback(StreamIdx idx,
                                           StreamPacketCallback cb) {
    hw_->setStreamPacketCallback(
        streamMap[static_cast<unsigned>(channel_)][idx], cb);
}
}

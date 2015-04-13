#include "Evil2Adapter.hpp"

namespace evil {

namespace {
const RegIdx fakeSystemControlReg = 0;
const RegIdx realSystemControlReg = 0;
const RegIdx fakeSystemConditionReg = 30;
const RegIdx realSystemConditionReg = 30;

const RegValue systemControlMask = 0b11111;
const unsigned systemControlBShift = 5;

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
    if (idx == fakeSystemControlReg) {
        auto val = hw_->readRegister(realSystemControlReg);
        if (channel_ == Channel::b) {
            val >>= systemControlBShift;
        }
        return val & systemControlMask;
    }

    if (idx == fakeSystemConditionReg) {
        auto val = hw_->readRegister(realSystemConditionReg);
        if (channel_ == Channel::b) {
            val >>= 1;
        }
        return val & 0b1;
    }

    if (channel_ == Channel::b) {
        idx += 11;
    }
    return hw_->readRegister(idx);
}

bool Evil2Adapter::modifyRegister(RegIdx idx, RegValue oldVal,
                                  RegValue newVal) {
    if (idx == fakeSystemConditionReg) return false;

    if (idx == fakeSystemControlReg) {
        // Figure out which part we need to modify.
        auto mask = systemControlMask;
        auto shift = 0;
        if (channel_ == Channel::b) {
            shift = systemControlBShift;
            mask <<= systemControlBShift;
        }

        // Extend oldVal/newVal with the unchanged part.
        auto oldReg = hw_->readRegister(realSystemControlReg);
        oldReg &= ~mask;
        oldReg |= (oldVal << shift);

        auto newReg = oldReg;
        newReg &= ~mask;
        newReg |= (newVal << shift);

        return hw_->modifyRegister(realSystemControlReg, oldReg, newReg);
    }

    if (channel_ == Channel::b) {
        idx += 11;
    }
    return hw_->modifyRegister(idx, oldVal, newVal);
}

void Evil2Adapter::addRegisterChangeCallback(RegisterChangeCallback cb) {
    hw_->addRegisterChangeCallback(cb);
}

StreamIdx Evil2Adapter::streamCount() { return 4; }

void Evil2Adapter::configureStream(StreamIdx idx, const StreamParams &params) {
    hw_->configureStream(streamMap[static_cast<unsigned>(channel_)][idx],
                         params);
}

void Evil2Adapter::setStreamPacketCallback(StreamIdx idx,
                                           StreamPacketCallback cb) {
    hw_->setStreamPacketCallback(
        streamMap[static_cast<unsigned>(channel_)][idx], cb);
}
}

// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "HoymilesRadio.h"
#include "commands/CommandAbstract.h"
#include "types.h"
#include <Arduino.h>
#include <cmt2300wrapper.h>
#include <memory>
#include <queue>
#include <vector>

// number of fragments hold in buffer
#define FRAGMENT_BUFFER_SIZE 30

#ifndef HOYMILES_CMT_WORK_FREQ
#define HOYMILES_CMT_WORK_FREQ 865000000
#endif

enum CountryModeId_t {
    MODE_EU,
    MODE_US,
    MODE_BR,
    CountryModeId_Max
};

struct CountryFrequencyDefinition_t {
    FrequencyBand_t Band;
    uint32_t Freq_Min;
    uint32_t Freq_Max;
    uint32_t Freq_Legal_Min;
    uint32_t Freq_Legal_Max;
    uint32_t Freq_Default;
    uint32_t Freq_StartUp;
};

struct CountryFrequencyList_t {
    CountryModeId_t mode;
    CountryFrequencyDefinition_t definition;
};

class HoymilesRadio_CMT : public HoymilesRadio {
public:
    void init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const int8_t pin_gpio2, const int8_t pin_gpio3);
    void loop();
    void setPALevel(const int8_t paLevel);
    void setInverterTargetFrequency(const uint32_t frequency);
    uint32_t getInverterTargetFrequency() const;

    bool isConnected() const;

    uint32_t getMinFrequency() const;
    uint32_t getMaxFrequency() const;
    static constexpr uint32_t getChannelWidth()
    {
        return FH_OFFSET * CMT2300A_ONE_STEP_SIZE;
    }

    CountryModeId_t getCountryMode() const;
    void setCountryMode(const CountryModeId_t mode);

    uint32_t getInvBootFrequency() const;

    uint32_t getFrequencyFromChannel(const uint8_t channel) const;
    uint8_t getChannelFromFrequency(const uint32_t frequency) const;

    std::vector<CountryFrequencyList_t> getCountryFrequencyList() const;

private:
    void ARDUINO_ISR_ATTR handleInt1();
    void ARDUINO_ISR_ATTR handleInt2();

    void sendEsbPacket(CommandAbstract& cmd);
    void loopThreeChannel();
    uint8_t threeChannelForSlot(const uint8_t slot) const;
    void switchThreeChannel(const uint8_t channel);
    void observeThreeChannelFragment(const fragment_t& fragment, const uint32_t receivedUs);
    void updateThreeChannelReception();

    // The CMT radio queues received fragments in _rxBuffer which are parsed
    // one per loop iteration in loop().
    virtual bool isRxBufferEmpty() const { return _rxBuffer.empty(); }

    std::unique_ptr<CMT2300A> _radio;

    volatile bool _packetReceived = false;
    volatile bool _packetSent = false;

    bool _gpio2_configured = false;
    bool _gpio3_configured = false;

    std::queue<fragment_t> _rxBuffer;
    TimeoutHelper _txTimeout;

    uint32_t _inverterTargetFrequency = HOYMILES_CMT_WORK_FREQ;
    int8_t _paLevel = 0;

    bool _threeChannelEnabled = false;
    bool _threeChannelBurst = false;
    uint8_t _threeChannelControl = 0;
    uint8_t _threeChannelPhase = 0xff;
    uint8_t _threeChannelLearnedSlot = 0xff;
    uint8_t _threeChannelNextFragment = 1;
    uint8_t _threeChannelRetryFragment = 0;
    uint8_t _threeChannelRetrySlot = 0;
    uint32_t _threeChannelStartedUs = 0;
    uint32_t _threeChannelWindowUs = 0;
    uint32_t _threeChannelTxCount = 0;
    uint32_t _threeChannelRxCount = 0;
    String _threeChannelCommand;
    struct {
        uint8_t fragmentId = 0;
        uint8_t channel = 0;
        uint32_t elapsedUs = 0;
    } _threeChannelTrace[12];
    uint8_t _threeChannelTraceCount = 0;

    bool cmtSwitchDtuFreq(const uint32_t to_frequency);

    CountryModeId_t _countryMode;
};

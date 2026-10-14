// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023-2026 Thomas Basler and others
 */
#include "HoymilesRadio_CMT.h"
#include "Hoymiles.h"
#include "HoymilesUtils.h"
#include "crc.h"
#include <FunctionalInterrupt.h>
#include <esp_log.h>
#include <frozen/map.h>

#undef TAG
static const char* TAG = "hoymiles";

constexpr CountryFrequencyDefinition_t make_value(FrequencyBand_t Band, uint32_t Freq_Legal_Min, uint32_t Freq_Legal_Max, uint32_t Freq_Default, uint32_t Freq_StartUp)
{
    // frequency can not be lower than actual initailized base freq + 250000
    uint32_t minFrequency = CMT2300A::getBaseFrequency(Band) + HoymilesRadio_CMT::getChannelWidth();

    // =923500, 0xFF does not work
    uint32_t maxFrequency = CMT2300A::getBaseFrequency(Band) + 0xFE * HoymilesRadio_CMT::getChannelWidth();

    CountryFrequencyDefinition_t v = { Band, minFrequency, maxFrequency, Freq_Legal_Min, Freq_Legal_Max, Freq_Default, Freq_StartUp };
    return v;
}

constexpr frozen::map<CountryModeId_t, CountryFrequencyDefinition_t, 3> countryDefinition = {
    { CountryModeId_t::MODE_EU, make_value(FrequencyBand_t::BAND_860, 863e6, 870e6, 865e6, 868e6) },
    { CountryModeId_t::MODE_US, make_value(FrequencyBand_t::BAND_900, 905e6, 925e6, 918e6, 915e6) },
    { CountryModeId_t::MODE_BR, make_value(FrequencyBand_t::BAND_900, 915e6, 928e6, 918e6, 915e6) },
};

uint32_t HoymilesRadio_CMT::getFrequencyFromChannel(const uint8_t channel) const
{
    return (_radio->getBaseFrequency() + channel * getChannelWidth());
}

uint8_t HoymilesRadio_CMT::getChannelFromFrequency(const uint32_t frequency) const
{
    if ((frequency % getChannelWidth()) != 0) {
        ESP_LOGE(TAG, "%.3f MHz is not divisible by %" PRIu32 " kHz!", frequency / 1000000.0, getChannelWidth());
        return 0xFF; // ERROR
    }
    if (frequency < getMinFrequency() || frequency > getMaxFrequency()) {
        ESP_LOGE(TAG, "%.2f MHz is out of Hoymiles/CMT range! (%.2f MHz - %.2f MHz)",
            frequency / 1000000.0, getMinFrequency() / 1000000.0, getMaxFrequency() / 1000000.0);
        return 0xFF; // ERROR
    }
    if (frequency < countryDefinition.at(_countryMode).Freq_Legal_Min || frequency > countryDefinition.at(_countryMode).Freq_Legal_Max) {
        ESP_LOGE(TAG, "!!! caution: %.2f MHz is out of region legal range! (%" PRIu32 " - %" PRIu32 " MHz)",
            frequency / 1000000.0,
            static_cast<uint32_t>(countryDefinition.at(_countryMode).Freq_Legal_Min / 1e6),
            static_cast<uint32_t>(countryDefinition.at(_countryMode).Freq_Legal_Max / 1e6));
    }

    return (frequency - _radio->getBaseFrequency()) / getChannelWidth(); // frequency to channel
}

std::vector<CountryFrequencyList_t> HoymilesRadio_CMT::getCountryFrequencyList() const
{
    std::vector<CountryFrequencyList_t> v;
    for (const auto& [key, value] : countryDefinition) {
        CountryFrequencyList_t s;
        s.mode = key;
        s.definition.Band = value.Band;
        s.definition.Freq_Default = value.Freq_Default;
        s.definition.Freq_StartUp = value.Freq_StartUp;
        s.definition.Freq_Min = value.Freq_Min;
        s.definition.Freq_Max = value.Freq_Max;
        s.definition.Freq_Legal_Max = value.Freq_Legal_Max;
        s.definition.Freq_Legal_Min = value.Freq_Legal_Min;

        v.push_back(s);
    }
    return v;
}

bool HoymilesRadio_CMT::cmtSwitchDtuFreq(const uint32_t to_frequency)
{
    const uint8_t toChannel = getChannelFromFrequency(to_frequency);
    if (toChannel == 0xFF) {
        return false;
    }

    _radio->setChannel(toChannel);

    return true;
}

void HoymilesRadio_CMT::init(const int8_t pin_sdio, const int8_t pin_clk, const int8_t pin_cs, const int8_t pin_fcs, const int8_t pin_gpio2, const int8_t pin_gpio3)
{
    _dtuSerial.u64 = 0;

    _radio.reset(new CMT2300A(pin_sdio, pin_clk, pin_cs, pin_fcs));

    _radio->begin();

    setCountryMode(CountryModeId_t::MODE_EU);
    cmtSwitchDtuFreq(_inverterTargetFrequency); // start dtu at work freqency, for fast Rx if inverter is already on and frequency switched

    if (!_radio->isChipConnected()) {
        ESP_LOGE(TAG, "CMT2300A: Connection error!!");
        return;
    }
    ESP_LOGI(TAG, "CMT2300A: Connection successful");

    if (pin_gpio2 >= 0) {
        attachInterrupt(digitalPinToInterrupt(pin_gpio2), std::bind(&HoymilesRadio_CMT::handleInt1, this), RISING);
        _gpio2_configured = true;
    }

    if (pin_gpio3 >= 0) {
        attachInterrupt(digitalPinToInterrupt(pin_gpio3), std::bind(&HoymilesRadio_CMT::handleInt2, this), RISING);
        _gpio3_configured = true;
    }

    _isInitialized = true;
}

void HoymilesRadio_CMT::loop()
{
    if (!_isInitialized) {
        return;
    }

    if (_threeChannelEnabled) {
        loopThreeChannel();
        return;
    }

    if (!_gpio3_configured) {
        if (_radio->rxFifoAvailable()) { // read INT2, PKT_OK flag
            _packetReceived = true;
        }
    }

    if (_packetReceived) {
        ESP_LOGV(TAG, "Interrupt received");
        while (_radio->available()) {
            if (_rxBuffer.size() > FRAGMENT_BUFFER_SIZE) {
                ESP_LOGE(TAG, "CMT2300A: Buffer full");
                _radio->flush_rx();
                continue;
            }

            fragment_t f;
            memset(f.fragment, 0xcc, MAX_RF_PAYLOAD_SIZE);
            f.len = std::min<uint8_t>(_radio->getDynamicPayloadSize(), MAX_RF_PAYLOAD_SIZE);
            f.channel = _radio->getChannel();
            f.rssi = _radio->getRssiDBm();
            f.wasReceived = false;
            f.mainCmd = 0x00;
            _radio->read(f.fragment, f.len);
            _rxBuffer.push(f);
        }
        _radio->flush_rx();
        _packetReceived = false;

    } else {
        // Perform package parsing only if no packages are received
        if (!_rxBuffer.empty()) {
            fragment_t f = _rxBuffer.front();
            if (checkFragmentCrc(f)) {

                const serial_u dtuId = convertSerialToRadioId(_dtuSerial);

                // The CMT RF module does not filter foreign packages by itself.
                // Has to be done manually here.
                if (memcmp(&f.fragment[5], &dtuId.b[1], 4) == 0) {

                    std::shared_ptr<InverterAbstract> inv = Hoymiles.getInverterByFragment(f);

                    if (nullptr != inv) {
                        // Save packet in inverter rx buffer
                        ESP_LOGD(TAG, "RX %.2f MHz --> %s | %" PRId8 " dBm",
                            getFrequencyFromChannel(f.channel) / 1000000.0, HoymilesUtils::dumpArray(f.fragment, f.len).c_str(), f.rssi);

                        inv->addRxFragment(f.fragment, f.len, f.rssi);
                    } else {
                        ESP_LOGE(TAG, "Inverter Not found!");
                    }
                }

            } else {
                ESP_LOGW(TAG, "Frame kaputt"); // ;-)
            }

            // Remove paket from buffer even it was corrupted
            _rxBuffer.pop();
        }
    }

    handleReceivedPackage();
}

uint8_t HoymilesRadio_CMT::threeChannelForSlot(const uint8_t slot) const
{
    const int8_t offsets[] = { 0, 1, -1 };
    const int16_t channel = static_cast<int16_t>(_threeChannelControl) + offsets[slot % 3];
    if (channel < 1 || channel > 0xfe) {
        return _threeChannelControl;
    }
    const uint32_t frequency = getFrequencyFromChannel(static_cast<uint8_t>(channel));
    const auto& definition = countryDefinition.at(_countryMode);
    if (frequency < definition.Freq_Legal_Min || frequency > definition.Freq_Legal_Max) {
        return _threeChannelControl;
    }
    return static_cast<uint8_t>(channel);
}

void HoymilesRadio_CMT::switchThreeChannel(const uint8_t channel)
{
    if (_radio->getChannel() == channel) {
        return;
    }
    if (_packetReceived || _radio->rxFifoAvailable()) {
        _packetReceived = true;
        return;
    }
    _radio->stopListening();
    _radio->setChannel(channel);
    if (!_radio->startListening()) {
        ESP_LOGE(TAG, "CMT 3CH: Failed to start receiver");
    }
}

void HoymilesRadio_CMT::observeThreeChannelFragment(const fragment_t& fragment, const uint32_t receivedUs)
{
    if (fragment.len < 11 || fragment.fragment[0] != 0x95 || !checkFragmentCrc(fragment)) {
        return;
    }
    const serial_u dtuId = convertSerialToRadioId(_dtuSerial);
    const uint32_t sender = (static_cast<uint32_t>(fragment.fragment[1]) << 24)
        | (static_cast<uint32_t>(fragment.fragment[2]) << 16)
        | (static_cast<uint32_t>(fragment.fragment[3]) << 8) | fragment.fragment[4];
    if (sender != 0xa02936ec || memcmp(&fragment.fragment[5], &dtuId.b[1], 4) != 0) {
        return;
    }
    const uint8_t fragmentId = fragment.fragment[9] & 0x7f;
    if (fragmentId == 0 || fragmentId >= MAX_RF_FRAGMENT_COUNT) {
        return;
    }
    uint8_t observedSlot = 0xff;
    for (uint8_t slot = 0; slot < 3; slot++) {
        if (fragment.channel == threeChannelForSlot(slot)) {
            observedSlot = slot;
            break;
        }
    }
    if (observedSlot == 0xff) {
        return;
    }
    _threeChannelLearnedSlot = (observedSlot + 1) % 3;
    _threeChannelRxCount++;
    if (_threeChannelTraceCount < 12) {
        auto& trace = _threeChannelTrace[_threeChannelTraceCount++];
        trace.fragmentId = fragmentId;
        trace.channel = fragment.channel;
        trace.elapsedUs = receivedUs - _threeChannelStartedUs;
    }
    if (!_threeChannelBurst) {
        return;
    }
    _threeChannelPhase = (observedSlot + 3 - ((fragmentId - 1) % 3)) % 3;
    if ((fragment.fragment[9] & 0x80) != 0) {
        _threeChannelBurst = false;
        return;
    }
    _threeChannelNextFragment = fragmentId + 1;
    updateThreeChannelReception();
}

void HoymilesRadio_CMT::updateThreeChannelReception()
{
    if (!_threeChannelBurst) {
        return;
    }
    const uint32_t elapsedUs = static_cast<uint32_t>(micros()) - _threeChannelStartedUs;
    if (elapsedUs >= _threeChannelWindowUs) {
        _threeChannelBurst = false;
        return;
    }
    if (_threeChannelPhase == 0xff) {
        return;
    }
    while (_threeChannelNextFragment < MAX_RF_FRAGMENT_COUNT
        && elapsedUs > 48000U + (_threeChannelNextFragment - 1U) * 50000U + 18000U) {
        _threeChannelNextFragment++;
    }
    if (_threeChannelNextFragment >= MAX_RF_FRAGMENT_COUNT) {
        _threeChannelBurst = false;
        return;
    }
    switchThreeChannel(threeChannelForSlot((_threeChannelPhase + _threeChannelNextFragment - 1) % 3));
}

void HoymilesRadio_CMT::loopThreeChannel()
{
    if (_radio->rxFifoAvailable()) {
        _packetReceived = true;
    }
    if (!_packetReceived) {
        updateThreeChannelReception();
    }
    if (_packetReceived) {
        _packetReceived = false;
        while (_radio->rxFifoAvailable()) {
            if (_rxBuffer.size() >= FRAGMENT_BUFFER_SIZE) {
                break;
            }
            fragment_t fragment = {};
            const uint8_t reportedLength = _radio->getDynamicPayloadSize();
            if (reportedLength == 0) {
                _radio->flush_rx();
                break;
            }
            fragment.len = std::min<uint8_t>(reportedLength, MAX_RF_PAYLOAD_SIZE);
            fragment.channel = _radio->getChannel();
            fragment.rssi = _radio->getRssiDBm();
            _radio->read(fragment.fragment, fragment.len);
            if (reportedLength > MAX_RF_PAYLOAD_SIZE) {
                _radio->flush_rx();
                continue;
            }
            const uint32_t receivedUs = micros();
            observeThreeChannelFragment(fragment, receivedUs);
            _rxBuffer.push(fragment);
        }
    }
    while (!_rxBuffer.empty()) {
        const fragment_t fragment = _rxBuffer.front();
        _rxBuffer.pop();
        if (fragment.len < 11 || !checkFragmentCrc(fragment)) {
            continue;
        }
        const serial_u dtuId = convertSerialToRadioId(_dtuSerial);
        if (memcmp(&fragment.fragment[5], &dtuId.b[1], 4) != 0) {
            continue;
        }
        auto inverter = Hoymiles.getInverterByFragment(fragment);
        if (inverter != nullptr) {
            inverter->addRxFragment(fragment.fragment, fragment.len, fragment.rssi);
        }
    }
    updateThreeChannelReception();
    handleReceivedPackage();
    if (!_busyFlag) {
        _threeChannelBurst = false;
        _threeChannelEnabled = false;
        ESP_LOGD(TAG, "CMT 3CH summary: %s tx=%" PRIu32 " rx=%" PRIu32,
            _threeChannelCommand.c_str(), _threeChannelTxCount, _threeChannelRxCount);
        for (uint8_t index = 0; index < _threeChannelTraceCount; index++) {
            const auto& trace = _threeChannelTrace[index];
            ESP_LOGD(TAG, "CMT 3CH RX: fragment=%u listen=%.2f MHz elapsed=%" PRIu32 "us",
                static_cast<unsigned>(trace.fragmentId), getFrequencyFromChannel(trace.channel) / 1000000.0, trace.elapsedUs);
        }
    }
}

void HoymilesRadio_CMT::setPALevel(const int8_t paLevel)
{
    _paLevel = paLevel;
    if (!_isInitialized) {
        return;
    }

    if (_radio->setPALevel(paLevel)) {
        ESP_LOGI(TAG, "CMT TX power set to %" PRId8 " dBm", paLevel);
    } else {
        ESP_LOGE(TAG, "CMT TX power %" PRId8 " dBm is not defined! (min: -10 dBm, max: 20 dBm)", paLevel);
    }
}

void HoymilesRadio_CMT::setInverterTargetFrequency(const uint32_t frequency)
{
    _inverterTargetFrequency = frequency;
    if (!_isInitialized) {
        return;
    }
    // Switch channel while staying in RX — GoStby, change channel, GoRx
    _radio->stopListening();
    cmtSwitchDtuFreq(_inverterTargetFrequency);
    _radio->startListening();
}

uint32_t HoymilesRadio_CMT::getInverterTargetFrequency() const
{
    return _inverterTargetFrequency;
}

bool HoymilesRadio_CMT::isConnected() const
{
    if (!_isInitialized) {
        return false;
    }
    return _radio->isChipConnected();
}

uint32_t HoymilesRadio_CMT::getMinFrequency() const
{
    return countryDefinition.at(_countryMode).Freq_Min;
}

uint32_t HoymilesRadio_CMT::getMaxFrequency() const
{
    return countryDefinition.at(_countryMode).Freq_Max;
}

CountryModeId_t HoymilesRadio_CMT::getCountryMode() const
{
    return _countryMode;
}

void HoymilesRadio_CMT::setCountryMode(const CountryModeId_t mode)
{
    _countryMode = mode;
    if (!_isInitialized) {
        return;
    }

    // setFrequencyBand() re-initializes the chip and thereby resets the
    // channel, the PA level and the RX state to the register bank defaults.
    // Restore them afterwards, otherwise the radio stays asleep on the band
    // base frequency with default TX power until the next config change.
    _radio->setFrequencyBand(countryDefinition.at(mode).Band);

    if (_inverterTargetFrequency >= getMinFrequency() && _inverterTargetFrequency <= getMaxFrequency()) {
        cmtSwitchDtuFreq(_inverterTargetFrequency);
    }
    _radio->setPALevel(_paLevel);
    _radio->startListening();
}

uint32_t HoymilesRadio_CMT::getInvBootFrequency() const
{
    // Hoymiles boot/init frequency after power up inverter or connection lost for 15 min
    return countryDefinition.at(_countryMode).Freq_StartUp;
}

void ARDUINO_ISR_ATTR HoymilesRadio_CMT::handleInt1()
{
    _packetSent = true;
}

void ARDUINO_ISR_ATTR HoymilesRadio_CMT::handleInt2()
{
    _packetReceived = true;
}

void HoymilesRadio_CMT::sendEsbPacket(CommandAbstract& cmd)
{
    cmd.incrementSendCount();

    cmd.setRouterAddress(DtuSerial().u64);

    const uint8_t* payload = cmd.getDataPayload();
    const bool threeChannelTarget = cmd.getTargetAddress() == 0x1164A02936ECULL;
    const uint8_t requestedFragment = payload[9] & 0x7f;
    const bool fragmentRequest = cmd.getDataSize() == 11 && payload[0] == 0x15
        && (payload[9] & 0x80) != 0 && requestedFragment > 0 && requestedFragment < MAX_RF_FRAGMENT_COUNT;
    if (!_busyFlag) {
        _threeChannelEnabled = threeChannelTarget;
        if (_threeChannelEnabled) {
            _threeChannelTxCount = 0;
            _threeChannelRxCount = 0;
            _threeChannelTraceCount = 0;
            _threeChannelRetryFragment = 0;
            _threeChannelCommand = cmd.getCommandName();
        }
    }
    _threeChannelBurst = false;
    uint8_t receiveSlot = 0;
    if (threeChannelTarget) {
        const uint8_t controlChannel = getChannelFromFrequency(_inverterTargetFrequency);
        if (controlChannel != _threeChannelControl) {
            _threeChannelLearnedSlot = 0xff;
        }
        _threeChannelControl = controlChannel;
        if (_threeChannelControl == 0xff) {
            _threeChannelEnabled = false;
        } else {
            _threeChannelTxCount++;
            if (fragmentRequest) {
                if (_threeChannelRetryFragment != requestedFragment) {
                    _threeChannelRetryFragment = requestedFragment;
                    _threeChannelRetrySlot = _threeChannelLearnedSlot < 3 ? _threeChannelLearnedSlot : 0;
                }
                receiveSlot = _threeChannelRetrySlot;
            } else {
                _threeChannelRetryFragment = 0;
                receiveSlot = _threeChannelLearnedSlot < 3 ? _threeChannelLearnedSlot : 0;
            }
        }
    }
    _radio->stopListening();
    _packetReceived = false;
    cmtSwitchDtuFreq(_inverterTargetFrequency);

    if (cmd.getDataPayload()[0] == 0x56) { // @todo(tbnobody) Bad hack to identify ChannelChange Command
        cmtSwitchDtuFreq(getInvBootFrequency());
    }

    ESP_LOGD(TAG, "TX %s %.2f MHz --> %s",
        cmd.getCommandName().c_str(), getFrequencyFromChannel(_radio->getChannel()) / 1000000.0, cmd.dumpDataPayload().c_str());

    if (!_radio->write(cmd.getDataPayload(), cmd.getDataSize())) {
        ESP_LOGE(TAG, "TX SPI Timeout");
    }
    cmtSwitchDtuFreq(_inverterTargetFrequency);
    if (_threeChannelEnabled) {
        _radio->setChannel(threeChannelForSlot(receiveSlot));
    }
    _radio->startListening();
    if (_threeChannelEnabled) {
        _threeChannelStartedUs = micros();
        _threeChannelWindowUs = std::min<uint32_t>(cmd.getTimeout(), 900) * 1000U;
        _threeChannelPhase = 0xff;
        _threeChannelNextFragment = 1;
        _threeChannelBurst = !fragmentRequest && cmd.getDataSize() >= 12
            && payload[0] == 0x15 && payload[9] == 0x80;
    }
    _busyFlag = true;
    _rxTimeout.set(cmd.getTimeout());
}

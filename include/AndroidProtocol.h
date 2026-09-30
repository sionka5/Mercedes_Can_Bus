#pragma once

#include <Arduino.h>

class AndroidProtocolHandler {
public:
    virtual uint8_t onAndroidHeartbeat(
        const uint8_t* payload,
        uint8_t payloadLength
    ) = 0;

    virtual uint8_t onAndroidSetText(
        const uint8_t* payload,
        uint8_t payloadLength
    ) = 0;

    virtual uint8_t onAndroidSetTelephone(
        const uint8_t* payload,
        uint8_t payloadLength
    ) = 0;

    virtual uint8_t onAndroidSetAmbientMax(
        const uint8_t* payload,
        uint8_t payloadLength
    ) = 0;

    virtual ~AndroidProtocolHandler() = default;
};

class AndroidProtocol {
public:
    static constexpr uint8_t ACK_OK = 0x00;
    static constexpr uint8_t ACK_BAD_PAYLOAD = 0x01;
    static constexpr uint8_t ACK_UNKNOWN_TYPE = 0x02;
    static constexpr uint8_t ACK_RADIO_OFF = 0x03;

    static constexpr uint8_t PAGE_UNKNOWN = 0x00;
    static constexpr uint8_t PAGE_AUDIO = 0x01;
    static constexpr uint8_t PAGE_TELEPHONE = 0x02;
    static constexpr uint8_t PAGE_NAVIGATION = 0x03;

    explicit AndroidProtocol(Stream& serial);

    void setHandler(AndroidProtocolHandler* handler);
    void poll();

    void sendStatus(
        uint8_t keyState,
        bool radioEnabled,
        bool heartbeatAlive
    );

    void sendButton(uint8_t page, uint8_t button);
    
    // ESP -> Android: wysyłka telemetrii (RPM, Prędkość, PWM, Limit %)
    void sendTelemetry(
        uint16_t rpm,
        float speed,
        uint8_t currentAmbientPwm,
        uint8_t maxAmbientPercent
    );

private:
    static constexpr uint8_t SOF1 = 0xAA;
    static constexpr uint8_t SOF2 = 0x55;

    // Android -> ESP
    static constexpr uint8_t MSG_HEARTBEAT = 0x11;
    static constexpr uint8_t MSG_SET_TEXT = 0x12;
    static constexpr uint8_t MSG_SET_TELEPHONE = 0x13;
    static constexpr uint8_t MSG_SET_AMBIENT_MAX = 0x05;

    // ESP -> Android
    static constexpr uint8_t MSG_BUTTON = 0x20;
    static constexpr uint8_t MSG_STATUS = 0x21;
    static constexpr uint8_t MSG_TELEMETRY = 0x04;
    static constexpr uint8_t MSG_ACK = 0x7F;

    static constexpr uint8_t MAX_LENGTH = 180;

    Stream& serial_;
    AndroidProtocolHandler* handler_;

    uint8_t txSequence_;
    uint8_t parserState_;
    uint8_t packetLength_;
    uint8_t packetIndex_;
    uint8_t packetBuffer_[MAX_LENGTH + 1];

    static uint8_t crc8Update(uint8_t crc, uint8_t data);
    static uint8_t crc8Calculate(uint8_t length, const uint8_t* data);

    void sendPacket(
        uint8_t type,
        const uint8_t* payload,
        uint8_t payloadLength
    );

    void sendAck(
        uint8_t acknowledgedSequence,
        uint8_t acknowledgedType,
        uint8_t status
    );

    void handlePacket(
        uint8_t sequence,
        uint8_t type,
        const uint8_t* payload,
        uint8_t payloadLength
    );

    void resetParser();
};
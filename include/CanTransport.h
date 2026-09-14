#pragma once

#include <Arduino.h>
#include "driver/twai.h"

class CanFrameHandler {
public:
    virtual void onCanFrame(const twai_message_t& frame) = 0;
    virtual ~CanFrameHandler() = default;
};

class CanTransport {
public:
    enum class AckResult : uint8_t {
        Accepted,
        Retry,
        Timeout
    };

    CanTransport();

    void setFrameHandler(CanFrameHandler* handler);

    bool begin();
    void stop();

    void setTransmitEnabled(bool enabled);
    bool transmitEnabled() const;
    bool ready() const;

    void resetSequence();

    void poll();
    void delayWithRx(uint32_t durationMs);

    bool sendStartup();
    bool sendKeepalive();

    bool sendLogicalPacket(
        const uint8_t* payload,
        size_t payloadLength
    );

    uint32_t ackTimeoutCount() const;

private:
    CanFrameHandler* handler_;

    bool ready_;
    bool transmitEnabled_;

    uint8_t sequenceNibble_;
    uint32_t ackTimeoutCount_;

    void dispatch(const twai_message_t& frame);
    void drainBeforeSegment();

    bool sendRaw1A4(const uint8_t data[8]);

    AckResult waitForAck(
        uint8_t sequence,
        uint16_t timeoutMs
    );

    bool sendSegmentWithAck(
        const uint8_t frame[8],
        uint8_t sequence
    );

    static uint8_t acceptedAckFor(uint8_t sequence);
    static uint8_t retryAckFor(uint8_t sequence);
};

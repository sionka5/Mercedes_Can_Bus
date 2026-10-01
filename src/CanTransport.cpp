#include "CanTransport.h"

#include <string.h>

#include "Config.h"
#include "Watchdog.h"

CanTransport::CanTransport()
    : handler_(nullptr),
      ready_(false),
      transmitEnabled_(false),
      sequenceNibble_(0),
      ackTimeoutCount_(0) {}

void CanTransport::setFrameHandler(CanFrameHandler* handler) {
    handler_ = handler;
}

bool CanTransport::begin() {
    twai_general_config_t general =
        TWAI_GENERAL_CONFIG_DEFAULT(
            Config::CAN_TXD_PIN,
            Config::CAN_RXD_PIN,
            TWAI_MODE_NORMAL
        );

    general.tx_queue_len = 32;
    general.rx_queue_len = 64;

    twai_timing_config_t timing = {};
    timing.brp = 48;
    timing.tseg_1 = 15;
    timing.tseg_2 = 4;
    timing.sjw = 3;
    timing.triple_sampling = false;

    twai_filter_config_t filter =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (
        twai_driver_install(
            &general,
            &timing,
            &filter
        ) != ESP_OK
    ) {
        return false;
    }

    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    ready_ = true;
    return true;
}

void CanTransport::stop() {
    transmitEnabled_ = false;

    if (!ready_) {
        return;
    }

    twai_stop();
    twai_driver_uninstall();

    ready_ = false;
}

void CanTransport::setTransmitEnabled(bool enabled) {
    transmitEnabled_ = enabled;
}

bool CanTransport::transmitEnabled() const {
    return transmitEnabled_;
}

bool CanTransport::ready() const {
    return ready_;
}

void CanTransport::resetSequence() {
    sequenceNibble_ = 0;
}

uint32_t CanTransport::ackTimeoutCount() const {
    return ackTimeoutCount_;
}

void CanTransport::dispatch(const twai_message_t& frame) {
    if (handler_ != nullptr) {
        handler_->onCanFrame(frame);
    }
}

void CanTransport::poll() {
    if (!ready_) {
        return;
    }

    twai_message_t frame;

    while (twai_receive(&frame, 0) == ESP_OK) {
        dispatch(frame);
        Watchdog::feed();
    }
}

void CanTransport::delayWithRx(uint32_t durationMs) {
    const uint32_t deadline = millis() + durationMs;

    while (static_cast<int32_t>(millis() - deadline) < 0) {
        twai_message_t frame;

        if (
            ready_ &&
            twai_receive(
                &frame,
                pdMS_TO_TICKS(1)
            ) == ESP_OK
        ) {
            dispatch(frame);
        }

        Watchdog::feed();
        yield();
    }

    poll();
}

bool CanTransport::sendRaw1A4(const uint8_t data[8]) {
    if (!ready_ || !transmitEnabled_) {
        return false;
    }

    twai_message_t frame = {};

    frame.identifier = Config::RADIO_TO_CLUSTER_ID;
    frame.extd = 0;
    frame.rtr = 0;
    frame.data_length_code = 8;

    memcpy(frame.data, data, 8);

    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        if (
            twai_transmit(
                &frame,
                pdMS_TO_TICKS(20)
            ) == ESP_OK
        ) {
            return true;
        }

        delayWithRx(5);
    }

    return false;
}

bool CanTransport::sendStartup() {
    resetSequence();

    const uint8_t startup[8] = {
        0xA0, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };

    if (!sendRaw1A4(startup)) {
        return false;
    }

    delayWithRx(10);
    return sendKeepalive();
}

bool CanTransport::sendKeepalive() {
    const uint8_t keepalive[8] = {
        0xA1, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00
    };

    return sendRaw1A4(keepalive);
}

uint8_t CanTransport::acceptedAckFor(uint8_t sequence) {
    return static_cast<uint8_t>(
        0xB0 | ((sequence + 1) & 0x0F)
    );
}

uint8_t CanTransport::retryAckFor(uint8_t sequence) {
    return static_cast<uint8_t>(
        0x90 | (sequence & 0x0F)
    );
}

void CanTransport::drainBeforeSegment() {
    // Stare ramki są przekazywane kontrolerowi, ale nie mogą
    // zostać uznane za ACK nowego segmentu.
    poll();
}

CanTransport::AckResult CanTransport::waitForAck(
    uint8_t sequence,
    uint16_t timeoutMs
) {
    const uint8_t accepted = acceptedAckFor(sequence);
    const uint8_t retry = retryAckFor(sequence);

    const uint32_t deadline = millis() + timeoutMs;

    while (static_cast<int32_t>(millis() - deadline) < 0) {
        twai_message_t frame;

        if (
            twai_receive(
                &frame,
                pdMS_TO_TICKS(1)
            ) != ESP_OK
        ) {
            Watchdog::feed();
            continue;
        }

        dispatch(frame);

        if (
            frame.identifier != Config::CLUSTER_TO_RADIO_ID ||
            frame.data_length_code < 1
        ) {
            continue;
        }

        if (frame.data[0] == accepted) {
            return AckResult::Accepted;
        }

        if (frame.data[0] == retry) {
            return AckResult::Retry;
        }

        Watchdog::feed();
    }

    return AckResult::Timeout;
}

bool CanTransport::sendSegmentWithAck(
    const uint8_t frame[8],
    uint8_t sequence
) {
    for (
        uint8_t attempt = 0;
        attempt < Config::MAX_SEGMENT_ATTEMPTS;
        attempt++
    ) {
        drainBeforeSegment();

        if (!sendRaw1A4(frame)) {
            if (attempt + 1 < Config::MAX_SEGMENT_ATTEMPTS) {
                delayWithRx(Config::ACK_RETRY_DELAY_MS);
            }
            continue;
        }

        const AckResult result =
            waitForAck(
                sequence,
                Config::ACK_TIMEOUT_MS
            );

        if (result == AckResult::Accepted) {
            return true;
        }

        if (result == AckResult::Timeout) {
            ackTimeoutCount_++;
        }

        // Retry ACK 0x9n albo timeout:
        // powtarzamy ten sam segment z tym samym numerem.
        if (attempt + 1 < Config::MAX_SEGMENT_ATTEMPTS) {
            delayWithRx(Config::ACK_RETRY_DELAY_MS);
        }
    }

    return false;
}

bool CanTransport::sendLogicalPacket(
    const uint8_t* payload,
    size_t payloadLength
) {
    if (
        payload == nullptr ||
        payloadLength == 0 ||
        payloadLength % 7 != 0
    ) {
        return false;
    }

    const size_t segmentCount = payloadLength / 7;
    size_t offset = 0;

    for (
        size_t segmentIndex = 0;
        segmentIndex < segmentCount;
        segmentIndex++
    ) {
        const bool last =
            segmentIndex + 1 == segmentCount;

        const uint8_t sequence =
            sequenceNibble_ & 0x0F;

        uint8_t frame[8] = {};

        frame[0] = sequence;

        if (last) {
            frame[0] |= 0x10;
        }

        memcpy(&frame[1], &payload[offset], 7);

        if (!sendSegmentWithAck(frame, sequence)) {
            return false;
        }

        // Zwiększamy dopiero po prawidłowym ACK.
        sequenceNibble_ =
            static_cast<uint8_t>((sequenceNibble_ + 1) & 0x0F);

        offset += 7;

        if (!last) {
            delayWithRx(Config::FRAME_GAP_MS);
        }
    }

    return true;
}

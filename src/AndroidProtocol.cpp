#include "AndroidProtocol.h"

#include <string.h>

AndroidProtocol::AndroidProtocol(Stream& serial)
    : serial_(serial),
      handler_(nullptr),
      txSequence_(1),
      parserState_(0),
      packetLength_(0),
      packetIndex_(0),
      packetBuffer_{} {}

void AndroidProtocol::setHandler(
    AndroidProtocolHandler* handler
) {
    handler_ = handler;
}


uint8_t AndroidProtocol::crc8Update(
    uint8_t crc,
    uint8_t data
) {
    crc ^= data;

    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x80) {
            crc = static_cast<uint8_t>(
                (crc << 1) ^ 0x07
            );
        } else {
            crc <<= 1;
        }
    }

    return crc;
}

uint8_t AndroidProtocol::crc8Calculate(
    uint8_t length,
    const uint8_t* data
) {
    uint8_t crc = crc8Update(0, length);

    for (uint8_t index = 0; index < length; index++) {
        crc = crc8Update(crc, data[index]);
    }

    return crc;
}

void AndroidProtocol::sendPacket(
    uint8_t type,
    const uint8_t* payload,
    uint8_t payloadLength
) {
    if (payloadLength > MAX_LENGTH - 2) {
        return;
    }

    const uint8_t length =
        static_cast<uint8_t>(2 + payloadLength);

    uint8_t data[MAX_LENGTH];

    data[0] = txSequence_++;

    if (txSequence_ == 0) {
        txSequence_ = 1;
    }

    data[1] = type;

    for (uint8_t index = 0; index < payloadLength; index++) {
        data[2 + index] = payload[index];
    }

    const uint8_t crc =
        crc8Calculate(length, data);

    serial_.write(SOF1);
    serial_.write(SOF2);
    serial_.write(length);
    serial_.write(data, length);
    serial_.write(crc);
}

void AndroidProtocol::sendAck(
    uint8_t acknowledgedSequence,
    uint8_t acknowledgedType,
    uint8_t status
) {
    const uint8_t payload[3] = {
        acknowledgedSequence,
        acknowledgedType,
        status
    };

    sendPacket(MSG_ACK, payload, sizeof(payload));
}

void AndroidProtocol::sendStatus(
    uint8_t keyState,
    bool radioEnabled,
    bool heartbeatAlive
) {
    const uint8_t payload[3] = {
        keyState,
        static_cast<uint8_t>(radioEnabled ? 1 : 0),
        static_cast<uint8_t>(heartbeatAlive ? 1 : 0)
    };

    sendPacket(MSG_STATUS, payload, sizeof(payload));
}

void AndroidProtocol::sendButton(uint8_t page, uint8_t button) {
    const uint8_t payload[2] = {page, button};
    sendPacket(MSG_BUTTON, payload, sizeof(payload));
}

void AndroidProtocol::sendTelemetry(
    uint16_t rpm,
    float speed,
    uint8_t currentAmbientPwm,
    uint8_t maxAmbientPercent
) {
    uint8_t payload[8];
    memcpy(&payload[0], &rpm, 2);
    memcpy(&payload[2], &speed, 4);
    payload[6] = currentAmbientPwm;
    payload[7] = maxAmbientPercent;

    sendPacket(MSG_TELEMETRY, payload, sizeof(payload));
}

void AndroidProtocol::handlePacket(
    uint8_t sequence,
    uint8_t type,
    const uint8_t* payload,
    uint8_t payloadLength
) {
    if (type == MSG_ACK) {
        return;
    }

    uint8_t status = ACK_UNKNOWN_TYPE;

    if (handler_ != nullptr) {
        switch (type) {
            case MSG_HEARTBEAT:
                status = handler_->onAndroidHeartbeat(
                    payload,
                    payloadLength
                );
                break;

            case MSG_SET_TEXT:
                status = handler_->onAndroidSetText(
                    payload,
                    payloadLength
                );
                break;

            case MSG_SET_TELEPHONE:
                status = handler_->onAndroidSetTelephone(
                    payload,
                    payloadLength
                );
                break;

            case MSG_SET_AMBIENT_MAX:
                status = handler_->onAndroidSetAmbientMax(
                    payload,
                    payloadLength
                );
                break;

            default:
                status = ACK_UNKNOWN_TYPE;
                break;
        }
    }

    sendAck(sequence, type, status);
}

void AndroidProtocol::resetParser() {
    parserState_ = 0;
    packetLength_ = 0;
    packetIndex_ = 0;
}

void AndroidProtocol::poll() {
    while (serial_.available() > 0) {
        const uint8_t byte =
            static_cast<uint8_t>(serial_.read());

        switch (parserState_) {
            case 0:
                if (byte == SOF1) {
                    parserState_ = 1;
                }
                break;

            case 1:
                if (byte == SOF2) {
                    parserState_ = 2;
                } else if (byte == SOF1) {
                    parserState_ = 1;
                } else {
                    resetParser();
                }
                break;

            case 2:
                packetLength_ = byte;

                if (
                    packetLength_ < 2 ||
                    packetLength_ > MAX_LENGTH
                ) {
                    resetParser();
                    break;
                }

                packetIndex_ = 0;
                parserState_ = 3;
                break;

            case 3:
                if (packetIndex_ > MAX_LENGTH) {
                    resetParser();
                    break;
                }

                packetBuffer_[packetIndex_++] = byte;

                if (
                    packetIndex_ ==
                    static_cast<uint16_t>(packetLength_) + 1U
                ) {
                    const uint8_t receivedCrc =
                        packetBuffer_[packetLength_];

                    const uint8_t calculatedCrc =
                        crc8Calculate(
                            packetLength_,
                            packetBuffer_
                        );

                    if (receivedCrc == calculatedCrc) {
                        const uint8_t sequence =
                            packetBuffer_[0];

                        const uint8_t type =
                            packetBuffer_[1];

                        const uint8_t* payload =
                            packetBuffer_ + 2;

                        const uint8_t payloadLength =
                            static_cast<uint8_t>(
                                packetLength_ - 2
                            );

                        handlePacket(
                            sequence,
                            type,
                            payload,
                            payloadLength
                        );
                    }

                    resetParser();
                }
                break;

            default:
                resetParser();
                break;
        }
    }
}
#include "AudioPage.h"

#include <string.h>

#include "CanTransport.h"
#include "Config.h"
#include "DisplayPayloads.h"
#include "TextEncoding.h"

AudioPage::AudioPage(CanTransport& transport)
    : transport_(transport) {}

bool AudioPage::show(
    const char* header,
    const char* body
) {
    if (!sendHeader(header)) {
        return false;
    }

    transport_.delayWithRx(
        Config::AUDIO_HEADER_BODY_GAP_MS
    );

    return sendBody(body);
}

bool AudioPage::sendHeader(const char* text) {
    uint8_t packet[DisplayPayloads::AUDIO_HEADER_LENGTH];

    memcpy(
        packet,
        DisplayPayloads::AUDIO_HEADER,
        sizeof(packet)
    );

    TextEncoding::copyNullTerminated(
        &packet[DisplayPayloads::AUDIO_HEADER_TEXT_OFFSET],
        DisplayPayloads::AUDIO_HEADER_TEXT_AREA,
        text,
        DisplayPayloads::AUDIO_HEADER_MAX_TEXT
    );

    return transport_.sendLogicalPacket(
        packet,
        sizeof(packet)
    );
}

bool AudioPage::sendBody(const char* text) {
    uint8_t packet[DisplayPayloads::AUDIO_BODY_LENGTH];

    memcpy(
        packet,
        DisplayPayloads::AUDIO_BODY,
        sizeof(packet)
    );

    TextEncoding::copyNullTerminated(
        &packet[DisplayPayloads::AUDIO_BODY_TEXT_OFFSET],
        DisplayPayloads::AUDIO_BODY_TEXT_AREA,
        text,
        DisplayPayloads::AUDIO_BODY_MAX_TEXT
    );

    return transport_.sendLogicalPacket(
        packet,
        sizeof(packet)
    );
}

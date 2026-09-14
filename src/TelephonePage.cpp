#include "TelephonePage.h"

#include <string.h>

#include "CanTransport.h"
#include "Config.h"
#include "DisplayPayloads.h"
#include "TextEncoding.h"

TelephonePage::TelephonePage(CanTransport& transport)
    : transport_(transport) {}

static size_t safeLength(const char* text, size_t maximum) {
    if (text == nullptr) {
        return 0;
    }

    size_t length = 0;
    while (length < maximum && text[length] != '\0') {
        length++;
    }
    return length;
}

bool TelephonePage::show(
    const char* body1,
    const char* body2,
    const char* body3
) {
    // Format potwierdzony z oryginalnego radia:
    // 72 01 + BODY1[5] + BODY2 + 0D + BODY3 + 00 + padding do 7 bajtów.
    constexpr size_t MAX_LOGICAL_LENGTH = 49;
    constexpr size_t BODY1_LENGTH = 5;

    const size_t body2Length = safeLength(body2, 30);
    const size_t body3Length = safeLength(body3, 30);

    const size_t rawLength =
        2 + BODY1_LENGTH + body2Length + 1 + body3Length + 1;

    const size_t packetLength = ((rawLength + 6) / 7) * 7;

    if (packetLength > MAX_LOGICAL_LENGTH) {
        return false;
    }

    uint8_t packet[MAX_LOGICAL_LENGTH] = {};
    packet[0] = 0x72;
    packet[1] = 0x01;

    // BODY1 ma dokładnie 5 bajtów i jest wyrównane do prawej.
    memset(&packet[2], 0x20, BODY1_LENGTH);
    const size_t body1Length = safeLength(body1, BODY1_LENGTH);
    for (size_t index = 0; index < body1Length; index++) {
        packet[2 + BODY1_LENGTH - body1Length + index] =
            TextEncoding::encodeChar(body1[index]);
    }

    size_t offset = 2 + BODY1_LENGTH;

    for (size_t index = 0; index < body2Length; index++) {
        packet[offset++] = TextEncoding::encodeChar(body2[index]);
    }

    packet[offset++] = 0x0D;

    for (size_t index = 0; index < body3Length; index++) {
        packet[offset++] = TextEncoding::encodeChar(body3[index]);
    }

    packet[offset] = 0x00;

    if (
        !transport_.sendLogicalPacket(
            DisplayPayloads::TEL_CONTROL,
            DisplayPayloads::STATUS_CONTROL_LENGTH
        )
    ) {
        return false;
    }

    transport_.delayWithRx(Config::PAGE_PACKET_GAP_MS);

    if (
        !transport_.sendLogicalPacket(
            DisplayPayloads::TEL_HEADER,
            DisplayPayloads::STATUS_HEADER_LENGTH
        )
    ) {
        return false;
    }

    transport_.delayWithRx(Config::PAGE_PACKET_GAP_MS);

    if (!transport_.sendLogicalPacket(packet, packetLength)) {
        return false;
    }

    if (Config::TEL_SEND_CLOSING_CONTROL) {
        transport_.delayWithRx(Config::TEL_CLOSE_DELAY_MS);

        if (
            !transport_.sendLogicalPacket(
                DisplayPayloads::TEL_CONTROL,
                DisplayPayloads::STATUS_CONTROL_LENGTH
            )
        ) {
            return false;
        }
    }

    return true;
}

bool TelephonePage::showReady(bool ready) {
    return show(
        "",
        ready ? "READY" : "OFF",
        ""
    );
}

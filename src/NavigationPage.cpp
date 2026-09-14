#include "NavigationPage.h"

#include <string.h>

#include "CanTransport.h"
#include "Config.h"
#include "DisplayPayloads.h"
#include "TextEncoding.h"

NavigationPage::NavigationPage(CanTransport& transport)
    : transport_(transport) {}

bool NavigationPage::showReady(bool ready) {
    uint8_t body[DisplayPayloads::STATUS_BODY_LENGTH];

    memcpy(
        body,
        DisplayPayloads::NAV_BODY,
        sizeof(body)
    );

    TextEncoding::copyFixed(
        &body[10],
        5,
        ready ? "READY" : "OFF",
        0x00
    );

    if (
        !transport_.sendLogicalPacket(
            DisplayPayloads::NAV_CONTROL,
            DisplayPayloads::STATUS_CONTROL_LENGTH
        )
    ) {
        return false;
    }

    transport_.delayWithRx(Config::PAGE_PACKET_GAP_MS);

    if (
        !transport_.sendLogicalPacket(
            DisplayPayloads::NAV_HEADER,
            DisplayPayloads::STATUS_HEADER_LENGTH
        )
    ) {
        return false;
    }

    transport_.delayWithRx(Config::PAGE_PACKET_GAP_MS);

    if (!transport_.sendLogicalPacket(body, sizeof(body))) {
        return false;
    }

    // Końcowe 6F jest domyślnie wyłączone,
    // bo potwierdzone testem kasowało treść NAV.
    if (Config::NAV_SEND_CLOSING_CONTROL) {
        transport_.delayWithRx(Config::PAGE_PACKET_GAP_MS);

        if (
            !transport_.sendLogicalPacket(
                DisplayPayloads::NAV_CONTROL,
                DisplayPayloads::STATUS_CONTROL_LENGTH
            )
        ) {
            return false;
        }
    }

    return true;
}

#pragma once

#include <Arduino.h>

namespace DisplayPayloads {

// ======================================================
// AUDIO
// ======================================================

constexpr size_t AUDIO_HEADER_LENGTH = 28;
constexpr size_t AUDIO_HEADER_TEXT_OFFSET = 2;
constexpr size_t AUDIO_HEADER_TEXT_AREA = 26;
constexpr size_t AUDIO_HEADER_MAX_TEXT = 11;

inline constexpr uint8_t AUDIO_HEADER[AUDIO_HEADER_LENGTH] = {
    0x11, // [00] opcode HEADER AUDIO
    0x01, // [01] format; testowano 0x40 lewo, 0x80 prawo

    0x00, // [02] początek tekstu
    0x00, // [03]
    0x00, // [04]
    0x00, // [05]
    0x00, // [06]
    0x00, // [07]
    0x00, // [08]
    0x00, // [09]
    0x00, // [10]
    0x00, // [11]
    0x00, // [12]
    0x00, // [13]
    0x00, // [14]
    0x00, // [15]
    0x00, // [16]
    0x00, // [17]
    0x00, // [18]
    0x00, // [19]
    0x00, // [20]
    0x00, // [21]
    0x00, // [22]
    0x00, // [23]
    0x00, // [24]
    0x00, // [25]
    0x00, // [26]
    0x00  // [27]
};

constexpr size_t AUDIO_BODY_LENGTH = 21;
constexpr size_t AUDIO_BODY_TEXT_OFFSET = 7;
constexpr size_t AUDIO_BODY_TEXT_AREA = 14;
constexpr size_t AUDIO_BODY_MAX_TEXT = 10;

inline constexpr uint8_t AUDIO_BODY[AUDIO_BODY_LENGTH] = {
    0x12, // [00] opcode BODY AUDIO
    0x11, // [01] format
    0x01, // [02] 01 none, 02 next, 03 previous
    0x01, // [03] nieznane / stałe
    0x01, // [04] nieznane / stałe
    0x01, // [05] nieznane / stałe
    0x01, // [06] nieznane / stałe

    0x00, // [07] początek tekstu
    0x00, // [08]
    0x00, // [09]
    0x00, // [10]
    0x00, // [11]
    0x00, // [12]
    0x00, // [13]
    0x00, // [14]
    0x00, // [15]
    0x00, // [16]
    0x00, // [17]
    0x00, // [18]
    0x00, // [19]
    0x00  // [20]
};

// ======================================================
// TEL
// ======================================================

constexpr size_t STATUS_CONTROL_LENGTH = 14;
constexpr size_t STATUS_HEADER_LENGTH = 7;
constexpr size_t STATUS_BODY_LENGTH = 21;

inline constexpr uint8_t TEL_CONTROL[STATUS_CONTROL_LENGTH] = {
    0x7F, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

inline constexpr uint8_t TEL_HEADER[STATUS_HEADER_LENGTH] = {
    0x71, 0x01,
    'T', 'E', 'L',
    0x00, 0x00
};

inline constexpr uint8_t TEL_BODY[STATUS_BODY_LENGTH] = {
    0x72, 0x01,

    0x20, 0x20, 0x20, 0x20, 0x20, // [02..06] status / ikony
    0x20, 0x20,                   // [07..08] BODY2
    0x0D,                         // [09] separator
    0x00, 0x00, 0x00, 0x00, 0x00, // [10..14] BODY3

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ======================================================
// NAV
// ======================================================

inline constexpr uint8_t NAV_CONTROL[STATUS_CONTROL_LENGTH] = {
    0x6F, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

inline constexpr uint8_t NAV_HEADER[STATUS_HEADER_LENGTH] = {
    0x61, 0x01,
    'N', 'A', 'V',
    0x00, 0x00
};

inline constexpr uint8_t NAV_BODY[STATUS_BODY_LENGTH] = {
    0x62, 0x01,

    0x20, 0x20, 0x20, 0x20, 0x20, // [02..06] status / ikony
    0x20, 0x20,                   // [07..08] BODY2
    0x0D,                         // [09] separator
    0x00, 0x00, 0x00, 0x00, 0x00, // [10..14] BODY3

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

} // namespace DisplayPayloads

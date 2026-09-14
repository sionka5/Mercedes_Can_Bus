#pragma once

#include <Arduino.h>
#include <stddef.h>

namespace TextEncoding {

uint8_t encodeChar(char character);

void copyNullTerminated(
    uint8_t* destination,
    size_t destinationLength,
    const char* text,
    size_t maximumTextLength
);

void copyFixed(
    uint8_t* destination,
    size_t fieldLength,
    const char* text,
    uint8_t padding
);

void copyAsciiToCharBuffer(
    char* destination,
    size_t destinationSize,
    const uint8_t* source,
    uint8_t sourceLength,
    const char* fallback
);

} // namespace TextEncoding

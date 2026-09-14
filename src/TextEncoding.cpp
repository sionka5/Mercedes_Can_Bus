#include "TextEncoding.h"

#include <string.h>

namespace TextEncoding {

uint8_t encodeChar(char character) {
    if (character >= 'a' && character <= 'z') {
        character = static_cast<char>(character - 32);
    }

    const uint8_t value = static_cast<uint8_t>(character);

    if (value < 0x20 || value > 0x7E) {
        return 0x20;
    }

    return value;
}

void copyNullTerminated(
    uint8_t* destination,
    size_t destinationLength,
    const char* text,
    size_t maximumTextLength
) {
    if (destination == nullptr || destinationLength == 0) {
        return;
    }

    memset(destination, 0x00, destinationLength);

    if (text == nullptr) {
        return;
    }

    size_t length = 0;

    while (
        text[length] != '\0' &&
        length < maximumTextLength &&
        length + 1 < destinationLength
    ) {
        destination[length] = encodeChar(text[length]);
        length++;
    }
}

void copyFixed(
    uint8_t* destination,
    size_t fieldLength,
    const char* text,
    uint8_t padding
) {
    if (destination == nullptr || fieldLength == 0) {
        return;
    }

    memset(destination, padding, fieldLength);

    if (text == nullptr) {
        return;
    }

    for (
        size_t index = 0;
        index < fieldLength && text[index] != '\0';
        index++
    ) {
        destination[index] = encodeChar(text[index]);
    }
}

void copyAsciiToCharBuffer(
    char* destination,
    size_t destinationSize,
    const uint8_t* source,
    uint8_t sourceLength,
    const char* fallback
) {
    if (destination == nullptr || destinationSize == 0) {
        return;
    }

    if (sourceLength == 0 || source == nullptr) {
        strncpy(destination, fallback, destinationSize - 1);
        destination[destinationSize - 1] = '\0';
        return;
    }

    size_t length = sourceLength;

    if (length > destinationSize - 1) {
        length = destinationSize - 1;
    }

    for (size_t index = 0; index < length; index++) {
        char character = static_cast<char>(source[index]);
        const uint8_t value = static_cast<uint8_t>(character);

        if (value < 0x20 || value > 0x7E) {
            character = ' ';
        }

        destination[index] = character;
    }

    destination[length] = '\0';
}

} // namespace TextEncoding

#pragma once

class CanTransport;

class AudioPage {
public:
    explicit AudioPage(CanTransport& transport);

    bool show(
        const char* header,
        const char* body
    );

private:
    CanTransport& transport_;

    bool sendHeader(const char* text);
    bool sendBody(const char* text);
};

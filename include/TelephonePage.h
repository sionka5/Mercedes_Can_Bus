#pragma once

class CanTransport;

class TelephonePage {
public:
    explicit TelephonePage(CanTransport& transport);

    // BODY1: krótkie pole 5 bajtów, wyrównane w prawo.
    // BODY2/BODY3: normalne linie, rozdzielone 0x0D.
    bool show(
        const char* body1,
        const char* body2,
        const char* body3
    );

    bool showReady(bool ready);

private:
    CanTransport& transport_;
};

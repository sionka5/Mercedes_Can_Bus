#pragma once

class CanTransport;

class NavigationPage {
public:
    explicit NavigationPage(CanTransport& transport);

    bool showReady(bool ready);

private:
    CanTransport& transport_;
};

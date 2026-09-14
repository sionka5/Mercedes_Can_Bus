#pragma once

class PowerManager {
public:
    void begin();
    void enterTransceiverSleep();
    void cutPower();

private:
    bool cuttingPower_ = false;
};

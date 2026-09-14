#include "PowerManager.h"

#include <Arduino.h>

#include "Config.h"
#include "Watchdog.h"

void PowerManager::begin() {
    pinMode(Config::BUCK_ENABLE_PIN, OUTPUT);
    digitalWrite(Config::BUCK_ENABLE_PIN, HIGH);

    pinMode(Config::TJA_STB_PIN, OUTPUT);
    pinMode(Config::TJA_EN_PIN, OUTPUT);

    digitalWrite(Config::TJA_STB_PIN, LOW);
    digitalWrite(Config::TJA_EN_PIN, LOW);
    delay(20);

    // Normal mode w używanym układzie: EN=HIGH, STB=HIGH.
    digitalWrite(Config::TJA_STB_PIN, HIGH);
    digitalWrite(Config::TJA_EN_PIN, HIGH);
    delay(20);
}

void PowerManager::enterTransceiverSleep() {
    pinMode(static_cast<int>(Config::CAN_TXD_PIN), INPUT_PULLUP);
    pinMode(static_cast<int>(Config::CAN_RXD_PIN), INPUT);

    // Go-to-sleep command.
    digitalWrite(Config::TJA_EN_PIN, HIGH);
    digitalWrite(Config::TJA_STB_PIN, LOW);
    delay(2);

    // Sleep.
    digitalWrite(Config::TJA_EN_PIN, LOW);
    digitalWrite(Config::TJA_STB_PIN, LOW);
    delay(20);
}

void PowerManager::cutPower() {
    if (cuttingPower_) {
        return;
    }

    cuttingPower_ = true;

    enterTransceiverSleep();
    delay(50);

    digitalWrite(Config::BUCK_ENABLE_PIN, LOW);

    // Zasilanie powinno zniknąć. Jeżeli nie zniknie,
    // pozostajemy bez aktywności.
    while (true) {
        Watchdog::feed();
        delay(1000);
    }
}

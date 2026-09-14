#include "VehicleController.h"

#include <string.h>

#include "Config.h"
#include "TextEncoding.h"
#include "Watchdog.h"

VehicleController::VehicleController()
    : power_(),
      can_(),
      android_(Serial),
      audio_(can_),
      telephone_(can_),
      navigation_(can_),
      keyState_(KeyState::Unknown),
      radioEnabled_(false),
      heartbeatAlive_(false),
      startupPending_(false),
      clusterRequestedKeepalive_(false),
      refreshMask_(RefreshNone),
      activePage_(AndroidProtocol::PAGE_UNKNOWN),
      bootMs_(0),
      lastCanActivityMs_(0),
      lastHeartbeatMs_(0),
      nextKeepaliveMs_(0),
      buttonsArmAtMs_(0),
      lastButtonEventMs_(0),
      buttonPressedAtMs_(0),
      lastButtonRepeatMs_(0),
      lastTelephoneButtonMs_(0),
      lastButton_(BUTTON_NONE),
      seenButtonSinceArm_(false),
      currentHeader_{},
      currentBody_{},
      telephoneBody1_{},
      telephoneBody2_{},
      telephoneBody3_{} {

    can_.setFrameHandler(this);
    android_.setHandler(this);

    setAudioOff();
    setTelephoneReady(false);
}

bool VehicleController::due(uint32_t timestamp) {
    return static_cast<int32_t>(
        millis() - timestamp
    ) >= 0;
}

bool VehicleController::elapsed(
    uint32_t since,
    uint32_t interval
) {
    return static_cast<uint32_t>(
        millis() - since
    ) >= interval;
}

void VehicleController::begin() {
    Serial.begin(115200);
    delay(800);

    Watchdog::begin();
    Watchdog::feed();

    bootMs_ = millis();
    lastCanActivityMs_ = millis();

    power_.begin();

    if (!can_.begin()) {
        delay(1000);
        shutdown();
        return;
    }

    sendStatus();
}

void VehicleController::service() {
    Watchdog::feed();

    // Najpierw szybko odbieramy dane z Androida i CAN.
    android_.poll();
    can_.poll();

    checkHeartbeatTimeout();
    serviceDisplays();

    if (
        radioEnabled_ &&
        clusterRequestedKeepalive_
    ) {
        clusterRequestedKeepalive_ = false;
        can_.sendKeepalive();
        nextKeepaliveMs_ =
            millis() + Config::KEEPALIVE_MS;
    }

    if (
        radioEnabled_ &&
        due(nextKeepaliveMs_)
    ) {
        can_.sendKeepalive();
        nextKeepaliveMs_ =
            millis() + Config::KEEPALIVE_MS;
    }

    can_.poll();
    android_.poll();

    checkSleep();

    Watchdog::feed();
    yield();
}

void VehicleController::onCanFrame(
    const twai_message_t& frame
) {
    lastCanActivityMs_ = millis();

    parseKeyFrame(frame);
    parseSteeringButton(frame);
    parseTelephoneButton(frame);

    if (
        frame.identifier ==
            Config::CLUSTER_TO_RADIO_ID &&
        frame.data_length_code >= 1 &&
        frame.data[0] == 0xA3
    ) {
        // Nie wysyłamy A1 z callbacka.
        // Tylko ustawiamy flagę, by uniknąć rekurencji.
        clusterRequestedKeepalive_ = true;
    }
}

uint8_t VehicleController::onAndroidHeartbeat(
    const uint8_t* payload,
    uint8_t payloadLength
) {
    (void)payload;

    if (payloadLength != 0) {
        return AndroidProtocol::ACK_BAD_PAYLOAD;
    }

    heartbeatArrived();
    return AndroidProtocol::ACK_OK;
}

uint8_t VehicleController::onAndroidSetText(
    const uint8_t* payload,
    uint8_t payloadLength
) {
    if (!heartbeatAlive_) {
        return AndroidProtocol::ACK_RADIO_OFF;
    }

    if (payload == nullptr || payloadLength < 2) {
        return AndroidProtocol::ACK_BAD_PAYLOAD;
    }

    const uint8_t headerLength = payload[0];
    const uint8_t bodyLength = payload[1];

    const uint16_t expectedLength =
        static_cast<uint16_t>(2) +
        headerLength +
        bodyLength;

    if (expectedLength != payloadLength) {
        return AndroidProtocol::ACK_BAD_PAYLOAD;
    }

    const uint8_t* header = payload + 2;
    const uint8_t* body = header + headerLength;

    TextEncoding::copyAsciiToCharBuffer(
        currentHeader_,
        sizeof(currentHeader_),
        header,
        headerLength,
        "CONNECTED"
    );

    TextEncoding::copyAsciiToCharBuffer(
        currentBody_,
        sizeof(currentBody_),
        body,
        bodyLength,
        "NO META"
    );

    // Android zmienia obecnie tylko AUDIO.
    requestRefresh(RefreshAudio);

    return AndroidProtocol::ACK_OK;
}


uint8_t VehicleController::onAndroidSetTelephone(
    const uint8_t* payload,
    uint8_t payloadLength
) {
    if (!heartbeatAlive_) {
        return AndroidProtocol::ACK_RADIO_OFF;
    }

    if (payload == nullptr || payloadLength < 3) {
        return AndroidProtocol::ACK_BAD_PAYLOAD;
    }

    const uint8_t body1Length = payload[0];
    const uint8_t body2Length = payload[1];
    const uint8_t body3Length = payload[2];

    const uint16_t expectedLength =
        static_cast<uint16_t>(3) +
        body1Length +
        body2Length +
        body3Length;

    if (
        expectedLength != payloadLength ||
        body1Length > 5 ||
        body2Length > 30 ||
        body3Length > 30
    ) {
        return AndroidProtocol::ACK_BAD_PAYLOAD;
    }

    const uint8_t* body1 = payload + 3;
    const uint8_t* body2 = body1 + body1Length;
    const uint8_t* body3 = body2 + body2Length;

    TextEncoding::copyAsciiToCharBuffer(
        telephoneBody1_,
        sizeof(telephoneBody1_),
        body1,
        body1Length,
        ""
    );

    TextEncoding::copyAsciiToCharBuffer(
        telephoneBody2_,
        sizeof(telephoneBody2_),
        body2,
        body2Length,
        "READY"
    );

    TextEncoding::copyAsciiToCharBuffer(
        telephoneBody3_,
        sizeof(telephoneBody3_),
        body3,
        body3Length,
        ""
    );

    requestRefresh(RefreshTelephone);
    return AndroidProtocol::ACK_OK;
}

void VehicleController::requestRefresh(uint8_t mask) {
    refreshMask_ |= mask;
}

void VehicleController::serviceDisplays() {
    if (!radioEnabled_) {
        startupPending_ = false;
        refreshMask_ = RefreshNone;
        return;
    }

    if (startupPending_) {
        if (!can_.sendStartup()) {
            return;
        }

        startupPending_ = false;
        can_.delayWithRx(60);
        requestRefresh(RefreshAll);
    }

    // Jedna strona na jedno wejście do serviceDisplays().
    // Dzięki temu połączenie z Androidem nie blokuje pętli
    // przez kilka stron naraz.
    if (refreshMask_ & RefreshAudio) {
        const bool ok = heartbeatAlive_
            ? audio_.show(currentHeader_, currentBody_)
            : audio_.show("RADIO", "OFF");

        if (ok) {
            refreshMask_ &=
                static_cast<uint8_t>(~RefreshAudio);
        }

        return;
    }

    if (refreshMask_ & RefreshNavigation) {
        if (navigation_.showReady(heartbeatAlive_)) {
            refreshMask_ &=
                static_cast<uint8_t>(~RefreshNavigation);
        }

        return;
    }

    if (refreshMask_ & RefreshTelephone) {
        const bool ok = heartbeatAlive_
            ? telephone_.show(
                telephoneBody1_,
                telephoneBody2_,
                telephoneBody3_
            )
            : telephone_.showReady(false);

        if (ok) {
            refreshMask_ &=
                static_cast<uint8_t>(~RefreshTelephone);
        }

        return;
    }
}

void VehicleController::setAudioOff() {
    strncpy(
        currentHeader_,
        "RADIO",
        sizeof(currentHeader_) - 1
    );
    currentHeader_[sizeof(currentHeader_) - 1] = '\0';

    strncpy(
        currentBody_,
        "OFF",
        sizeof(currentBody_) - 1
    );
    currentBody_[sizeof(currentBody_) - 1] = '\0';
}

void VehicleController::setAudioConnected() {
    strncpy(
        currentHeader_,
        "CONNECTED",
        sizeof(currentHeader_) - 1
    );
    currentHeader_[sizeof(currentHeader_) - 1] = '\0';

    strncpy(
        currentBody_,
        "NO META",
        sizeof(currentBody_) - 1
    );
    currentBody_[sizeof(currentBody_) - 1] = '\0';
}


void VehicleController::setTelephoneReady(bool ready) {
    telephoneBody1_[0] = '\0';

    strncpy(
        telephoneBody2_,
        ready ? "READY" : "OFF",
        sizeof(telephoneBody2_) - 1
    );
    telephoneBody2_[sizeof(telephoneBody2_) - 1] = '\0';

    telephoneBody3_[0] = '\0';
}

void VehicleController::heartbeatArrived() {
    lastHeartbeatMs_ = millis();

    if (heartbeatAlive_) {
        return;
    }

    heartbeatAlive_ = true;
    setAudioConnected();
    setTelephoneReady(true);

    // Tylko flagi. Brak transmisji CAN wewnątrz parsera Serial.
    requestRefresh(RefreshAll);
    sendStatus();
}

void VehicleController::heartbeatLost() {
    if (!heartbeatAlive_) {
        return;
    }

    heartbeatAlive_ = false;
    setAudioOff();
    setTelephoneReady(false);

    requestRefresh(RefreshAll);
    sendStatus();
}

void VehicleController::checkHeartbeatTimeout() {
    if (
        heartbeatAlive_ &&
        elapsed(
            lastHeartbeatMs_,
            Config::PC_HEARTBEAT_TIMEOUT_MS
        )
    ) {
        heartbeatLost();
    }
}

VehicleController::KeyState
VehicleController::decodeKeyState(uint8_t value) {
    switch (value) {
        case 0x00:
            return KeyState::Out;

        case 0x01:
            return KeyState::Inserted;

        case 0x03:
            return KeyState::Position1;

        case 0x0F:
            return KeyState::Ignition;

        default:
            return KeyState::Unknown;
    }
}

bool VehicleController::shouldRadioRun(KeyState state) {
    return (
        state == KeyState::Position1 ||
        state == KeyState::Ignition
    );
}

void VehicleController::parseKeyFrame(
    const twai_message_t& frame
) {
    if (
        frame.identifier != 0x000 ||
        frame.data_length_code < 1
    ) {
        return;
    }

    const KeyState newState =
        decodeKeyState(frame.data[0]);

    if (
        newState == KeyState::Unknown ||
        newState == keyState_
    ) {
        return;
    }

    const bool wasRunning =
        shouldRadioRun(keyState_);

    const bool nowRunning =
        shouldRadioRun(newState);

    keyState_ = newState;

    if (!wasRunning && nowRunning) {
        startRadio();
    } else if (wasRunning && !nowRunning) {
        stopRadio();
    }

    sendStatus();
}

void VehicleController::startRadio() {
    if (radioEnabled_) {
        return;
    }

    radioEnabled_ = true;

    can_.setTransmitEnabled(true);
    can_.resetSequence();

    clusterRequestedKeepalive_ = false;
    nextKeepaliveMs_ =
        millis() + Config::KEEPALIVE_MS;

    resetButtonState();
    activePage_ = AndroidProtocol::PAGE_UNKNOWN;

    startupPending_ = true;
    requestRefresh(RefreshAll);
}

void VehicleController::stopRadio() {
    if (!radioEnabled_) {
        return;
    }

    radioEnabled_ = false;
    can_.setTransmitEnabled(false);

    clusterRequestedKeepalive_ = false;
    nextKeepaliveMs_ = 0;

    startupPending_ = false;
    refreshMask_ = RefreshNone;

    buttonsArmAtMs_ = 0;
    lastButton_ = BUTTON_NONE;
    seenButtonSinceArm_ = false;
    activePage_ = AndroidProtocol::PAGE_UNKNOWN;
}

void VehicleController::resetButtonState() {
    buttonsArmAtMs_ =
        millis() + Config::BUTTON_ARM_DELAY_MS;

    lastButtonEventMs_ = 0;
    buttonPressedAtMs_ = 0;
    lastButtonRepeatMs_ = 0;
    lastTelephoneButtonMs_ = 0;

    lastButton_ = BUTTON_NONE;
    seenButtonSinceArm_ = false;
}

bool VehicleController::buttonsArmed() const {
    return (
        radioEnabled_ &&
        heartbeatAlive_ &&
        buttonsArmAtMs_ != 0 &&
        due(buttonsArmAtMs_)
    );
}

bool VehicleController::isRepeatableButton(uint8_t button) {
    return (
        button == BUTTON_VOLUME_UP ||
        button == BUTTON_VOLUME_DOWN
    );
}

void VehicleController::sendButton(
    uint8_t page,
    uint8_t button
) {
    if (!heartbeatAlive_) {
        return;
    }

    android_.sendButton(page, button);
}

void VehicleController::parseSteeringButton(
    const twai_message_t& frame
) {
    if (
        frame.identifier !=
            Config::CLUSTER_TO_RADIO_ID ||
        frame.data_length_code < 4
    ) {
        return;
    }

    // ACK Bx/9x nie może zostać uznany za przycisk.
    if (
        frame.data[0] != 0xAF ||
        frame.data[1] != 0x01
    ) {
        return;
    }

    activePage_ = frame.data[2];

    if (!buttonsArmed()) {
        return;
    }

    const uint8_t button = frame.data[3];
    const uint32_t now = millis();

    if (
        !seenButtonSinceArm_ &&
        button == BUTTON_RELEASE
    ) {
        lastButton_ = BUTTON_RELEASE;
        return;
    }

    if (button == BUTTON_RELEASE) {
        if (lastButton_ != BUTTON_RELEASE) {
            sendButton(activePage_, BUTTON_RELEASE);
        }

        lastButton_ = BUTTON_RELEASE;
        buttonPressedAtMs_ = 0;
        lastButtonRepeatMs_ = 0;
        return;
    }

    if (button == lastButton_) {
        if (
            isRepeatableButton(button) &&
            buttonPressedAtMs_ != 0 &&
            elapsed(
                buttonPressedAtMs_,
                Config::BUTTON_HOLD_DELAY_MS
            ) &&
            (
                lastButtonRepeatMs_ == 0 ||
                elapsed(
                    lastButtonRepeatMs_,
                    Config::BUTTON_REPEAT_MS
                )
            )
        ) {
            lastButtonRepeatMs_ = now;
            sendButton(activePage_, button);
        }

        return;
    }

    if (
        lastButtonEventMs_ != 0 &&
        now - lastButtonEventMs_ <
            Config::BUTTON_DEBOUNCE_MS
    ) {
        return;
    }

    lastButtonEventMs_ = now;
    buttonPressedAtMs_ = now;
    lastButtonRepeatMs_ = 0;
    lastButton_ = button;
    seenButtonSinceArm_ = true;

    sendButton(activePage_, button);
}

void VehicleController::parseTelephoneButton(
    const twai_message_t& frame
) {
    if (
        frame.identifier != 0x1A8 ||
        frame.data_length_code < 2 ||
        !buttonsArmed()
    ) {
        return;
    }

    const uint8_t b0 = frame.data[0];
    const uint8_t b1 = frame.data[1];
    const uint32_t now = millis();

    if (
        lastTelephoneButtonMs_ != 0 &&
        now - lastTelephoneButtonMs_ <
            Config::TEL_BUTTON_DEBOUNCE_MS
    ) {
        return;
    }

    if (b0 == 0x40 && b1 == 0x00) {
        lastTelephoneButtonMs_ = now;

        if (activePage_ == AndroidProtocol::PAGE_TELEPHONE) {
            sendButton(activePage_, BUTTON_PHONE_PICKUP);
        } else {
            sendButton(activePage_, BUTTON_BRIGHTNESS_UP);
        }
        return;
    }

    if (b0 == 0x80 && b1 == 0x00) {
        lastTelephoneButtonMs_ = now;

        if (activePage_ == AndroidProtocol::PAGE_TELEPHONE) {
            sendButton(activePage_, BUTTON_PHONE_HANGUP);
        } else {
            sendButton(activePage_, BUTTON_BRIGHTNESS_DOWN);
        }
    }
}

void VehicleController::sendStatus() {
    android_.sendStatus(
        static_cast<uint8_t>(keyState_),
        radioEnabled_,
        heartbeatAlive_
    );
}

void VehicleController::checkSleep() {
    if (
        !elapsed(
            bootMs_,
            Config::STARTUP_SLEEP_ARM_MS
        )
    ) {
        return;
    }

    if (keyState_ != KeyState::Out) {
        return;
    }

    if (
        elapsed(
            lastCanActivityMs_,
            Config::CAN_IDLE_SLEEP_MS
        )
    ) {
        shutdown();
    }
}

void VehicleController::shutdown() {
    radioEnabled_ = false;
    heartbeatAlive_ = false;

    can_.setTransmitEnabled(false);
    can_.stop();

    power_.cutPower();
}

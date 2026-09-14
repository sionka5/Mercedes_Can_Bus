#pragma once

#include <Arduino.h>
#include "driver/twai.h"

#include "AndroidProtocol.h"
#include "AudioPage.h"
#include "CanTransport.h"
#include "NavigationPage.h"
#include "PowerManager.h"
#include "TelephonePage.h"

class VehicleController
    : public CanFrameHandler,
      public AndroidProtocolHandler {
public:
    VehicleController();

    void begin();
    void service();

    void onCanFrame(const twai_message_t& frame) override;

    uint8_t onAndroidHeartbeat(
        const uint8_t* payload,
        uint8_t payloadLength
    ) override;

    uint8_t onAndroidSetText(
        const uint8_t* payload,
        uint8_t payloadLength
    ) override;

    uint8_t onAndroidSetTelephone(
        const uint8_t* payload,
        uint8_t payloadLength
    ) override;

private:
    enum class KeyState : uint8_t {
        Unknown = 0,
        Out = 1,
        Inserted = 2,
        Position1 = 3,
        Ignition = 4
    };

    enum RefreshMask : uint8_t {
        RefreshNone = 0x00,
        RefreshAudio = 0x01,
        RefreshTelephone = 0x02,
        RefreshNavigation = 0x04,
        RefreshAll = 0x07
    };

    static constexpr uint8_t BUTTON_RELEASE = 0x00;
    static constexpr uint8_t BUTTON_VOLUME_UP = 0x10;
    static constexpr uint8_t BUTTON_VOLUME_DOWN = 0x20;
    static constexpr uint8_t BUTTON_NONE = 0xFF;

    static constexpr uint8_t BUTTON_BRIGHTNESS_UP = 0x30;
    static constexpr uint8_t BUTTON_BRIGHTNESS_DOWN = 0x31;
    static constexpr uint8_t BUTTON_PHONE_PICKUP = 0x40;
    static constexpr uint8_t BUTTON_PHONE_HANGUP = 0x41;

    PowerManager power_;
    CanTransport can_;
    AndroidProtocol android_;

    AudioPage audio_;
    TelephonePage telephone_;
    NavigationPage navigation_;

    KeyState keyState_;

    bool radioEnabled_;
    bool heartbeatAlive_;
    bool startupPending_;
    bool clusterRequestedKeepalive_;

    uint8_t refreshMask_;
    uint8_t activePage_;

    uint32_t bootMs_;
    uint32_t lastCanActivityMs_;
    uint32_t lastHeartbeatMs_;
    uint32_t nextKeepaliveMs_;

    uint32_t buttonsArmAtMs_;
    uint32_t lastButtonEventMs_;
    uint32_t buttonPressedAtMs_;
    uint32_t lastButtonRepeatMs_;
    uint32_t lastTelephoneButtonMs_;

    uint8_t lastButton_;
    bool seenButtonSinceArm_;

    char currentHeader_[32];
    char currentBody_[32];

    char telephoneBody1_[6];
    char telephoneBody2_[40];
    char telephoneBody3_[40];

    static bool due(uint32_t timestamp);
    static bool elapsed(uint32_t since, uint32_t interval);

    static KeyState decodeKeyState(uint8_t value);
    static bool shouldRadioRun(KeyState state);

    void requestRefresh(uint8_t mask);
    void serviceDisplays();

    void startRadio();
    void stopRadio();

    void setAudioOff();
    void setAudioConnected();
    void setTelephoneReady(bool ready);

    void heartbeatArrived();
    void heartbeatLost();
    void checkHeartbeatTimeout();

    void parseKeyFrame(const twai_message_t& frame);
    void parseSteeringButton(const twai_message_t& frame);
    void parseTelephoneButton(const twai_message_t& frame);

    bool buttonsArmed() const;
    static bool isRepeatableButton(uint8_t button);
    void resetButtonState();
    void sendButton(uint8_t page, uint8_t button);

    void sendStatus();

    void checkSleep();
    void shutdown();
};

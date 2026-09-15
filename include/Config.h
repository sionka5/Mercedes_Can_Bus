#pragma once

#include <Arduino.h>
#include "driver/twai.h"

namespace Config {

// ESP32-C3 + TJA1053T
constexpr gpio_num_t CAN_TXD_PIN = GPIO_NUM_3;
constexpr gpio_num_t CAN_RXD_PIN = GPIO_NUM_2;

constexpr uint8_t BUCK_ENABLE_PIN = 0;
constexpr uint8_t TJA_STB_PIN = 6;
constexpr uint8_t TJA_EN_PIN = 7;

// CAN / radio
constexpr uint32_t RADIO_TO_CLUSTER_ID = 0x1A4;
constexpr uint32_t CLUSTER_TO_RADIO_ID = 0x1D0;

constexpr uint32_t KEEPALIVE_MS = 1000;
constexpr uint32_t DISPLAY_REFRESH_MS = 1500;
constexpr uint16_t FRAME_GAP_MS = 8;
constexpr uint16_t PAGE_PACKET_GAP_MS = 8;
constexpr uint16_t AUDIO_HEADER_BODY_GAP_MS = 80;

constexpr uint16_t ACK_TIMEOUT_MS = 60;
constexpr uint16_t ACK_RETRY_DELAY_MS = 115;
constexpr uint8_t MAX_SEGMENT_ATTEMPTS = 5;

// TEL
constexpr uint16_t TEL_CLOSE_DELAY_MS = 786;

// Końcowe 7F potrafi zamknąć nowszy ekran podczas szybkiego przewijania.
// Dla dynamicznej strony TEL pozostaje wyłączone.
constexpr bool TEL_SEND_CLOSING_CONTROL = false;

// Końcowe 6F kasuje NAV.
constexpr bool NAV_SEND_CLOSING_CONTROL = false;

// Android
constexpr uint32_t PC_HEARTBEAT_TIMEOUT_MS = 3000;

// Kluczyk / przyciski
constexpr uint32_t BUTTON_ARM_DELAY_MS = 1000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 300;
constexpr uint32_t BUTTON_HOLD_DELAY_MS = 450;
constexpr uint32_t BUTTON_REPEAT_MS = 120;
constexpr uint32_t TEL_BUTTON_DEBOUNCE_MS = 300;

// Zasilanie
constexpr uint32_t CAN_IDLE_SLEEP_MS = 5000;
constexpr uint32_t STARTUP_SLEEP_ARM_MS = 5000;

// Watchdog — domyślnie wyłączony dla pierwszych testów.
constexpr bool ENABLE_TASK_WATCHDOG = false;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 8000;

} // namespace Config

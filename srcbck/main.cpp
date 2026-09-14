#include <Arduino.h>
#include <string.h>

#include "driver/twai.h"
#include "esp_idf_version.h"
#include "esp_task_wdt.h"

// ======================================================
// ESP32-C3 + TJA1053T
// ======================================================

#define CAN_TXD_PIN        GPIO_NUM_3
#define CAN_RXD_PIN        GPIO_NUM_2

#define BUCK_ENABLE_PIN    0
#define TJA_STB_PIN        6
#define TJA_EN_PIN         7

// ======================================================
// KONFIGURACJA
// ======================================================

static const uint32_t KEEPALIVE_MS            = 1000;
static const uint32_t CAN_IDLE_SLEEP_MS       = 5000;
static const uint32_t STARTUP_SLEEP_ARM_MS    = 5000;
static const uint32_t PC_HEARTBEAT_TIMEOUT_MS = 3000;

static const uint32_t BUTTON_ARM_DELAY_MS      = 1000;
static const uint32_t BUTTON_DEBOUNCE_MS       = 300;
static const uint32_t TEL_BUTTON_DEBOUNCE_MS   = 300;

static const uint16_t FRAME_GAP_MS             = 8;
static const uint16_t PAGE_PACKET_GAP_MS       = 8;
static const uint16_t AUDIO_HEADER_BODY_GAP_MS = 80;

static const uint16_t ACK_TIMEOUT_MS           = 60;
static const uint16_t ACK_RETRY_DELAY_MS       = 115;
static const uint8_t  MAX_SEGMENT_ATTEMPTS     = 5;

static const uint16_t TEL_CLOSE_DELAY_MS       = 786;

// Oryginalne radio wysyłało końcowe 7F.
// Gdyby TEL READY/OFF znikało po chwili, zmień na false.
static const bool TEL_SEND_CLOSING_CONTROL = true;

// Końcowe 6F kasuje treść NAV — musi zostać false.
static const bool NAV_SEND_CLOSING_CONTROL = false;

static const uint32_t WATCHDOG_TIMEOUT_MS = 8000;

// ======================================================
// DŁUGOŚCI LOGICZNYCH PAKIETÓW
// ======================================================

// AUDIO
static const size_t AUDIO_HEADER_PACKET_LENGTH = 28; // 4 x 7
static const size_t AUDIO_HEADER_TEXT_OFFSET   = 2;
static const size_t AUDIO_HEADER_TEXT_AREA     = 26;
static const size_t AUDIO_HEADER_MAX_TEXT      = 11;

static const size_t AUDIO_BODY_PACKET_LENGTH = 21; // 3 x 7
static const size_t AUDIO_BODY_TEXT_OFFSET   = 7;
static const size_t AUDIO_BODY_TEXT_AREA     = 14;
static const size_t AUDIO_BODY_MAX_TEXT      = 10;

// TEL / NAV
static const size_t STATUS_CONTROL_PACKET_LENGTH = 14; // 2 x 7
static const size_t STATUS_HEADER_PACKET_LENGTH  = 7;  // 1 x 7
static const size_t STATUS_BODY_PACKET_LENGTH    = 21; // 3 x 7

// ======================================================
// CZYTELNE SZABLONY PAYLOADÓW
// ======================================================
//
// Indeksy dotyczą LOGICZNEGO payloadu, bez transportowego
// bajtu sekwencji. To jest główne miejsce do szybkiej
// podmiany bajtów i bitów podczas dalszych testów.

// -------------------------- AUDIO HEADER ----------------------------

static const uint8_t AUDIO_HEADER_TEMPLATE[AUDIO_HEADER_PACKET_LENGTH] = {
    0x11, // [00] opcode HEADER AUDIO
    0x01, // [01] format; testowano 0x40 lewo, 0x80 prawo

    0x00, // [02] początek tekstu
    0x00, // [03]
    0x00, // [04]
    0x00, // [05]
    0x00, // [06]
    0x00, // [07]
    0x00, // [08]
    0x00, // [09]
    0x00, // [10]
    0x00, // [11]
    0x00, // [12]
    0x00, // [13]
    0x00, // [14]
    0x00, // [15]
    0x00, // [16]
    0x00, // [17]
    0x00, // [18]
    0x00, // [19]
    0x00, // [20]
    0x00, // [21]
    0x00, // [22]
    0x00, // [23]
    0x00, // [24]
    0x00, // [25]
    0x00, // [26]
    0x00  // [27]
};

// --------------------------- AUDIO BODY -----------------------------

static const uint8_t AUDIO_BODY_TEMPLATE[AUDIO_BODY_PACKET_LENGTH] = {
    0x12, // [00] opcode BODY AUDIO
    0x11, // [01] format
    0x01, // [02] 01 brak symbolu, 02 next, 03 previous
    0x01, // [03] nieznane / stałe
    0x01, // [04] nieznane / stałe
    0x01, // [05] nieznane / stałe
    0x01, // [06] nieznane / stałe

    0x00, // [07] początek tekstu
    0x00, // [08]
    0x00, // [09]
    0x00, // [10]
    0x00, // [11]
    0x00, // [12]
    0x00, // [13]
    0x00, // [14]
    0x00, // [15]
    0x00, // [16]
    0x00, // [17]
    0x00, // [18]
    0x00, // [19]
    0x00  // [20]
};

// ------------------------------- TEL --------------------------------

static const uint8_t TEL_CONTROL_TEMPLATE[STATUS_CONTROL_PACKET_LENGTH] = {
    0x7F, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t TEL_HEADER_TEMPLATE[STATUS_HEADER_PACKET_LENGTH] = {
    0x71, 0x01,
    'T', 'E', 'L',
    0x00, 0x00
};

// BODY1 = 5-bajtowe pole statusu/ikon.
// BODY2 = 2 znaki.
// 0x0D = separator.
// BODY3 = 5 znaków.
static const uint8_t TEL_BODY_TEMPLATE[STATUS_BODY_PACKET_LENGTH] = {
    0x72, 0x01,

    0x20, 0x20, 0x20, 0x20, 0x20, // [02..06] status / ikony
    0x20, 0x20,                   // [07..08] BODY2
    0x0D,                         // [09] separator
    0x00, 0x00, 0x00, 0x00, 0x00, // [10..14] BODY3

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ------------------------------- NAV --------------------------------

static const uint8_t NAV_CONTROL_TEMPLATE[STATUS_CONTROL_PACKET_LENGTH] = {
    0x6F, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t NAV_HEADER_TEMPLATE[STATUS_HEADER_PACKET_LENGTH] = {
    0x61, 0x01,
    'N', 'A', 'V',
    0x00, 0x00
};

static const uint8_t NAV_BODY_TEMPLATE[STATUS_BODY_PACKET_LENGTH] = {
    0x62, 0x01,

    0x20, 0x20, 0x20, 0x20, 0x20, // [02..06] status / ikony
    0x20, 0x20,                   // [07..08] BODY2
    0x0D,                         // [09] separator
    0x00, 0x00, 0x00, 0x00, 0x00, // [10..14] BODY3

    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ======================================================
// PROTOKÓŁ SERIAL ANDROID <-> ESP
// ======================================================

static const uint8_t SOF1 = 0xAA;
static const uint8_t SOF2 = 0x55;

// Android -> ESP
static const uint8_t MSG_HEARTBEAT = 0x11;
static const uint8_t MSG_SET_TEXT  = 0x12;

// ESP -> Android
static const uint8_t MSG_BUTTON = 0x20;
static const uint8_t MSG_STATUS = 0x21;
static const uint8_t MSG_ACK    = 0x7F;

static const uint8_t ACK_OK           = 0x00;
static const uint8_t ACK_BAD_PAYLOAD  = 0x01;
static const uint8_t ACK_UNKNOWN_TYPE = 0x02;
static const uint8_t ACK_RADIO_OFF    = 0x03;

static const uint8_t PROTO_MAX_LEN = 180;

// ======================================================
// PRZYCISKI
// ======================================================

static const uint8_t BTN_RELEASE = 0x00;
static const uint8_t BTN_NONE    = 0xFF;

static const uint8_t BTN_BRIGHTNESS_UP   = 0x30;
static const uint8_t BTN_BRIGHTNESS_DOWN = 0x31;

// ======================================================
// STAN KLUCZYKA
// ======================================================

enum KeyState : uint8_t {
    KEY_UNKNOWN  = 0,
    KEY_OUT      = 1,
    KEY_INSERTED = 2,
    KEY_POS1     = 3,
    KEY_IGNITION = 4
};

// ======================================================
// WATCHDOG
// ======================================================

static void watchdogBegin() {
#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t config = {};
    config.timeout_ms = WATCHDOG_TIMEOUT_MS;
    config.idle_core_mask = 0;
    config.trigger_panic = true;

    esp_err_t result = esp_task_wdt_init(&config);

    if (result == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_reconfigure(&config);
    }
#else
    esp_task_wdt_init(WATCHDOG_TIMEOUT_MS / 1000, true);
#endif

    // Core może już mieć task loop dodany — wynik celowo ignorujemy.
    esp_task_wdt_add(nullptr);
}

static inline void watchdogFeed() {
    esp_task_wdt_reset();
}

// ======================================================
// CZAS
// ======================================================

static bool due(uint32_t timestamp) {
    return (int32_t)(millis() - timestamp) >= 0;
}

static bool elapsed(uint32_t since, uint32_t interval) {
    return (uint32_t)(millis() - since) >= interval;
}

// ======================================================
// CRC8
// ======================================================

static uint8_t crc8Update(uint8_t crc, uint8_t data) {
    crc ^= data;

    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 0x80) {
            crc = (uint8_t)((crc << 1) ^ 0x07);
        } else {
            crc <<= 1;
        }
    }

    return crc;
}

static uint8_t crc8Calculate(uint8_t length, const uint8_t* data) {
    uint8_t crc = crc8Update(0, length);

    for (uint8_t i = 0; i < length; i++) {
        crc = crc8Update(crc, data[i]);
    }

    return crc;
}

// ======================================================
// KODOWANIE TEKSTU
// ======================================================

static uint8_t encodeDisplayChar(char character) {
    if (character >= 'a' && character <= 'z') {
        character = (char)(character - 32);
    }

    if (
        (uint8_t)character < 0x20 ||
        (uint8_t)character > 0x7E
    ) {
        return 0x20;
    }

    return (uint8_t)character;
}

static void copyNullTerminatedText(
    uint8_t* destination,
    size_t destinationLength,
    const char* text,
    size_t maximumTextLength
) {
    memset(destination, 0x00, destinationLength);

    size_t length = 0;

    while (
        text[length] != '\0' &&
        length < maximumTextLength &&
        length + 1 < destinationLength
    ) {
        destination[length] = encodeDisplayChar(text[length]);
        length++;
    }
}

static void copyFixedText(
    uint8_t* destination,
    size_t fieldLength,
    const char* text,
    uint8_t padding
) {
    memset(destination, padding, fieldLength);

    for (
        size_t index = 0;
        index < fieldLength && text[index] != '\0';
        index++
    ) {
        destination[index] = encodeDisplayChar(text[index]);
    }
}

// ======================================================
// ZASILANIE / TJA1053T
// ======================================================

static void buckOn() {
    pinMode(BUCK_ENABLE_PIN, OUTPUT);
    digitalWrite(BUCK_ENABLE_PIN, HIGH);
}

static void buckOff() {
    digitalWrite(BUCK_ENABLE_PIN, LOW);
}

static void tjaPinsInit() {
    pinMode(TJA_STB_PIN, OUTPUT);
    pinMode(TJA_EN_PIN, OUTPUT);

    digitalWrite(TJA_STB_PIN, LOW);
    digitalWrite(TJA_EN_PIN, LOW);

    delay(20);
}

static void tjaNormalMode() {
    // W tym układzie normal mode wymaga EN=HIGH i STB=HIGH.
    digitalWrite(TJA_STB_PIN, HIGH);
    digitalWrite(TJA_EN_PIN, HIGH);

    delay(20);
}

static void tjaSleepMode() {
    pinMode((int)CAN_TXD_PIN, INPUT_PULLUP);
    pinMode((int)CAN_RXD_PIN, INPUT);

    // Go-to-sleep.
    digitalWrite(TJA_EN_PIN, HIGH);
    digitalWrite(TJA_STB_PIN, LOW);
    delay(2);

    // Sleep.
    digitalWrite(TJA_EN_PIN, LOW);
    digitalWrite(TJA_STB_PIN, LOW);
    delay(20);
}

// ======================================================
// ODBIORCA RAMEK
// ======================================================

class CanFrameSink {
public:
    virtual void onCanFrame(const twai_message_t& frame) = 0;
    virtual ~CanFrameSink() = default;
};

// ======================================================
// TRANSPORT LEGACY AUDIO 10
// ======================================================

class LegacyCanTransport {
public:
    enum AckResult : uint8_t {
        ACK_ACCEPTED,
        ACK_RETRY,
        ACK_TIMEOUT
    };

    LegacyCanTransport()
        : sink_(nullptr),
          enabled_(false),
          ready_(false),
          sequenceNibble_(0),
          ackTimeoutCount_(0) {}

    void setSink(CanFrameSink* sink) {
        sink_ = sink;
    }

    bool begin() {
        twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TXD_PIN,
            CAN_RXD_PIN,
            TWAI_MODE_NORMAL
        );

        general.tx_queue_len = 32;
        general.rx_queue_len = 64;

        twai_timing_config_t timing = {};
        timing.brp = 48;
        timing.tseg_1 = 15;
        timing.tseg_2 = 4;
        timing.sjw = 3;
        timing.triple_sampling = false;

        twai_filter_config_t filter =
            TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (
            twai_driver_install(
                &general,
                &timing,
                &filter
            ) != ESP_OK
        ) {
            return false;
        }

        if (twai_start() != ESP_OK) {
            twai_driver_uninstall();
            return false;
        }

        ready_ = true;
        return true;
    }

    void stop() {
        enabled_ = false;

        if (!ready_) {
            return;
        }

        twai_stop();
        twai_driver_uninstall();

        ready_ = false;
    }

    void setEnabled(bool enabled) {
        enabled_ = enabled;
    }

    bool isReady() const {
        return ready_;
    }

    uint32_t ackTimeoutCount() const {
        return ackTimeoutCount_;
    }

    void resetSessionSequence() {
        sequenceNibble_ = 0;
    }

    void poll() {
        if (!ready_) {
            return;
        }

        twai_message_t frame;

        while (twai_receive(&frame, 0) == ESP_OK) {
            dispatch(frame);
            watchdogFeed();
        }
    }

    void delayWithRx(uint32_t durationMs) {
        const uint32_t until = millis() + durationMs;

        while (!due(until)) {
            twai_message_t frame;

            if (
                ready_ &&
                twai_receive(
                    &frame,
                    pdMS_TO_TICKS(1)
                ) == ESP_OK
            ) {
                dispatch(frame);
            }

            watchdogFeed();
            yield();
        }

        poll();
    }

    bool sendStartup() {
        resetSessionSequence();

        const uint8_t frame[8] = {
            0xA0, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        };

        if (!sendRaw1A4(frame)) {
            return false;
        }

        delayWithRx(10);
        return sendKeepalive();
    }

    bool sendKeepalive() {
        const uint8_t frame[8] = {
            0xA1, 0x01, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        };

        return sendRaw1A4(frame);
    }

    bool sendLogicalPacket(
        const uint8_t* payload,
        size_t payloadLength
    ) {
        if (
            payload == nullptr ||
            payloadLength == 0 ||
            payloadLength % 7 != 0
        ) {
            return false;
        }

        const size_t frameCount = payloadLength / 7;
        size_t offset = 0;

        for (
            size_t frameIndex = 0;
            frameIndex < frameCount;
            frameIndex++
        ) {
            const bool lastFrame =
                frameIndex + 1 == frameCount;

            const uint8_t sequence =
                sequenceNibble_ & 0x0F;

            uint8_t frame[8] = {};

            frame[0] = sequence;

            if (lastFrame) {
                frame[0] |= 0x10;
            }

            memcpy(
                &frame[1],
                &payload[offset],
                7
            );

            if (!sendSegmentWithAck(frame, sequence)) {
                return false;
            }

            sequenceNibble_ =
                (uint8_t)((sequenceNibble_ + 1) & 0x0F);

            offset += 7;

            if (!lastFrame) {
                delayWithRx(FRAME_GAP_MS);
            }
        }

        return true;
    }

private:
    CanFrameSink* sink_;
    bool enabled_;
    bool ready_;
    uint8_t sequenceNibble_;
    uint32_t ackTimeoutCount_;

    void dispatch(const twai_message_t& frame) {
        if (sink_ != nullptr) {
            sink_->onCanFrame(frame);
        }
    }

    bool sendRaw1A4(const uint8_t data[8]) {
        if (!enabled_ || !ready_) {
            return false;
        }

        twai_message_t message = {};

        message.identifier = 0x1A4;
        message.extd = 0;
        message.rtr = 0;
        message.data_length_code = 8;

        memcpy(message.data, data, 8);

        // Krótkie ponowienie, jeśli kolejka TX jest chwilowo zajęta.
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            if (
                twai_transmit(
                    &message,
                    pdMS_TO_TICKS(20)
                ) == ESP_OK
            ) {
                return true;
            }

            delayWithRx(5);
        }

        return false;
    }

    static uint8_t acceptedAckFor(uint8_t sequence) {
        return (uint8_t)(
            0xB0 | ((sequence + 1) & 0x0F)
        );
    }

    static uint8_t retryAckFor(uint8_t sequence) {
        return (uint8_t)(
            0x90 | (sequence & 0x0F)
        );
    }

    AckResult waitForSegmentAck(
        uint8_t sequence,
        uint16_t timeoutMs
    ) {
        const uint8_t acceptedAck =
            acceptedAckFor(sequence);

        const uint8_t retryAck =
            retryAckFor(sequence);

        const uint32_t until =
            millis() + timeoutMs;

        while (!due(until)) {
            twai_message_t frame;

            if (
                twai_receive(
                    &frame,
                    pdMS_TO_TICKS(1)
                ) != ESP_OK
            ) {
                watchdogFeed();
                continue;
            }

            dispatch(frame);

            if (
                frame.identifier != 0x1D0 ||
                frame.data_length_code < 1
            ) {
                continue;
            }

            if (frame.data[0] == acceptedAck) {
                return ACK_ACCEPTED;
            }

            if (frame.data[0] == retryAck) {
                return ACK_RETRY;
            }
        }

        return ACK_TIMEOUT;
    }

    bool sendSegmentWithAck(
        const uint8_t frame[8],
        uint8_t sequence
    ) {
        for (
            uint8_t attempt = 1;
            attempt <= MAX_SEGMENT_ATTEMPTS;
            attempt++
        ) {
            // Usuwamy stare ramki, aby nie przyjąć ACK
            // poprzedniego segmentu.
            poll();

            if (!sendRaw1A4(frame)) {
                if (attempt < MAX_SEGMENT_ATTEMPTS) {
                    delayWithRx(ACK_RETRY_DELAY_MS);
                }
                continue;
            }

            const AckResult result =
                waitForSegmentAck(
                    sequence,
                    ACK_TIMEOUT_MS
                );

            if (result == ACK_ACCEPTED) {
                return true;
            }

            if (result == ACK_TIMEOUT) {
                ackTimeoutCount_++;
            }

            // ACK 0x9n albo timeout:
            // powtarzamy dokładnie ten sam segment i numer.
            if (attempt < MAX_SEGMENT_ATTEMPTS) {
                delayWithRx(ACK_RETRY_DELAY_MS);
            }
        }

        return false;
    }
};

// ======================================================
// STRONA AUDIO
// ======================================================

class AudioPage {
public:
    explicit AudioPage(LegacyCanTransport& transport)
        : transport_(transport) {}

    bool show(
        const char* header,
        const char* body
    ) {
        if (!sendHeader(header)) {
            return false;
        }

        transport_.delayWithRx(
            AUDIO_HEADER_BODY_GAP_MS
        );

        return sendBody(body);
    }

private:
    LegacyCanTransport& transport_;

    bool sendHeader(const char* text) {
        uint8_t packet[AUDIO_HEADER_PACKET_LENGTH];

        memcpy(
            packet,
            AUDIO_HEADER_TEMPLATE,
            sizeof(packet)
        );

        copyNullTerminatedText(
            &packet[AUDIO_HEADER_TEXT_OFFSET],
            AUDIO_HEADER_TEXT_AREA,
            text,
            AUDIO_HEADER_MAX_TEXT
        );

        return transport_.sendLogicalPacket(
            packet,
            sizeof(packet)
        );
    }

    bool sendBody(const char* text) {
        uint8_t packet[AUDIO_BODY_PACKET_LENGTH];

        memcpy(
            packet,
            AUDIO_BODY_TEMPLATE,
            sizeof(packet)
        );

        copyNullTerminatedText(
            &packet[AUDIO_BODY_TEXT_OFFSET],
            AUDIO_BODY_TEXT_AREA,
            text,
            AUDIO_BODY_MAX_TEXT
        );

        return transport_.sendLogicalPacket(
            packet,
            sizeof(packet)
        );
    }
};

// ======================================================
// STRONA STATUSOWA TEL / NAV
// ======================================================

class StatusPage {
public:
    StatusPage(
        LegacyCanTransport& transport,
        const uint8_t* controlTemplate,
        const uint8_t* headerTemplate,
        const uint8_t* bodyTemplate,
        bool sendClosingControl
    )
        : transport_(transport),
          controlTemplate_(controlTemplate),
          headerTemplate_(headerTemplate),
          bodyTemplate_(bodyTemplate),
          sendClosingControl_(sendClosingControl) {}

    bool showReady(bool ready) {
        return showBottomLine(
            ready ? "READY" : "OFF"
        );
    }

private:
    LegacyCanTransport& transport_;
    const uint8_t* controlTemplate_;
    const uint8_t* headerTemplate_;
    const uint8_t* bodyTemplate_;
    bool sendClosingControl_;

    bool showBottomLine(const char* text) {
        uint8_t body[STATUS_BODY_PACKET_LENGTH];

        memcpy(
            body,
            bodyTemplate_,
            sizeof(body)
        );

        // BODY3: normalne pięcioznakowe pole tekstowe.
        copyFixedText(
            &body[10],
            5,
            text,
            0x00
        );

        if (
            !transport_.sendLogicalPacket(
                controlTemplate_,
                STATUS_CONTROL_PACKET_LENGTH
            )
        ) {
            return false;
        }

        transport_.delayWithRx(
            PAGE_PACKET_GAP_MS
        );

        if (
            !transport_.sendLogicalPacket(
                headerTemplate_,
                STATUS_HEADER_PACKET_LENGTH
            )
        ) {
            return false;
        }

        transport_.delayWithRx(
            PAGE_PACKET_GAP_MS
        );

        if (
            !transport_.sendLogicalPacket(
                body,
                sizeof(body)
            )
        ) {
            return false;
        }

        if (sendClosingControl_) {
            transport_.delayWithRx(
                TEL_CLOSE_DELAY_MS
            );

            if (
                !transport_.sendLogicalPacket(
                    controlTemplate_,
                    STATUS_CONTROL_PACKET_LENGTH
                )
            ) {
                return false;
            }
        }

        return true;
    }
};

// ======================================================
// KONTROLER CAŁEGO UKŁADU
// ======================================================

class VehicleDisplayController : public CanFrameSink {
public:
    VehicleDisplayController()
        : audioPage_(can_),
          telPage_(
              can_,
              TEL_CONTROL_TEMPLATE,
              TEL_HEADER_TEMPLATE,
              TEL_BODY_TEMPLATE,
              TEL_SEND_CLOSING_CONTROL
          ),
          navPage_(
              can_,
              NAV_CONTROL_TEMPLATE,
              NAV_HEADER_TEMPLATE,
              NAV_BODY_TEMPLATE,
              NAV_SEND_CLOSING_CONTROL
          ),
          keyState_(KEY_UNKNOWN),
          radioTxEnabled_(false),
          pcHeartbeatAlive_(false),
          shuttingDown_(false),
          clusterRequestedKeepalive_(false),
          startupPending_(false),
          refreshMask_(REFRESH_NONE),
          lastPcHeartbeatMs_(0),
          nextKeepaliveMs_(0),
          lastCanActivityMs_(0),
          bootMs_(0),
          buttonsArmAtMs_(0),
          lastButtonEventMs_(0),
          lastTelButtonMs_(0),
          lastButtonKey_(BTN_NONE),
          seenRealButtonSinceArm_(false),
          protoTxSequence_(1),
          serialState_(0),
          serialLength_(0),
          serialIndex_(0) {

        setAudioOffText();
        can_.setSink(this);
    }

    void begin() {
        Serial.begin(115200);
        delay(800);

        watchdogBegin();
        watchdogFeed();

        bootMs_ = millis();
        lastCanActivityMs_ = millis();

        buckOn();
        tjaPinsInit();
        tjaNormalMode();

        if (!can_.begin()) {
            delay(1000);
            powerOffEsp();
        }

        sendStatus();
    }

    void service() {
        watchdogFeed();

        readSerialProtocol();
        can_.poll();

        checkPcHeartbeat();
        serviceDisplays();

        if (
            radioTxEnabled_ &&
            clusterRequestedKeepalive_
        ) {
            clusterRequestedKeepalive_ = false;
            can_.sendKeepalive();
            nextKeepaliveMs_ =
                millis() + KEEPALIVE_MS;
        }

        if (
            radioTxEnabled_ &&
            due(nextKeepaliveMs_)
        ) {
            can_.sendKeepalive();
            nextKeepaliveMs_ =
                millis() + KEEPALIVE_MS;
        }

        can_.poll();
        readSerialProtocol();

        checkCanSleep();

        watchdogFeed();
        yield();
    }

    void onCanFrame(
        const twai_message_t& frame
    ) override {
        lastCanActivityMs_ = millis();

        parseKeyFrame(frame);
        parseSteeringButtonFrame(frame);
        parseTelButtonFrame(frame);

        if (
            frame.identifier == 0x1D0 &&
            frame.data_length_code >= 1 &&
            frame.data[0] == 0xA3
        ) {
            clusterRequestedKeepalive_ = true;
        }
    }

private:
    enum RefreshMask : uint8_t {
        REFRESH_NONE  = 0x00,
        REFRESH_AUDIO = 0x01,
        REFRESH_TEL   = 0x02,
        REFRESH_NAV   = 0x04,
        REFRESH_ALL   = 0x07
    };

    LegacyCanTransport can_;
    AudioPage audioPage_;
    StatusPage telPage_;
    StatusPage navPage_;

    KeyState keyState_;

    bool radioTxEnabled_;
    bool pcHeartbeatAlive_;
    bool shuttingDown_;

    bool clusterRequestedKeepalive_;
    bool startupPending_;

    uint8_t refreshMask_;

    uint32_t lastPcHeartbeatMs_;
    uint32_t nextKeepaliveMs_;
    uint32_t lastCanActivityMs_;
    uint32_t bootMs_;

    uint32_t buttonsArmAtMs_;
    uint32_t lastButtonEventMs_;
    uint32_t lastTelButtonMs_;

    uint8_t lastButtonKey_;
    bool seenRealButtonSinceArm_;

    char currentHeader_[32];
    char currentBody_[32];

    uint8_t protoTxSequence_;

    uint8_t serialState_;
    uint8_t serialLength_;
    uint8_t serialIndex_;
    uint8_t serialBuffer_[PROTO_MAX_LEN + 1];

    // --------------------------------------------------
    // ZASILANIE
    // --------------------------------------------------

    void powerOffEsp() {
        if (shuttingDown_) {
            return;
        }

        shuttingDown_ = true;

        radioTxEnabled_ = false;
        pcHeartbeatAlive_ = false;

        can_.setEnabled(false);
        can_.stop();

        tjaSleepMode();

        delay(50);
        buckOff();

        // Jeżeli zasilanie z jakiegoś powodu nie zniknie,
        // nie pozwalamy watchdogowi resetować ESP w pętli.
        while (true) {
            watchdogFeed();
            delay(1000);
        }
    }

    // --------------------------------------------------
    // WYŚWIETLACZE
    // --------------------------------------------------

    void requestRefresh(uint8_t mask) {
        refreshMask_ |= mask;
    }

    void setAudioOffText() {
        strncpy(
            currentHeader_,
            "RADIO",
            sizeof(currentHeader_) - 1
        );
        currentHeader_[
            sizeof(currentHeader_) - 1
        ] = '\0';

        strncpy(
            currentBody_,
            "OFF",
            sizeof(currentBody_) - 1
        );
        currentBody_[
            sizeof(currentBody_) - 1
        ] = '\0';
    }

    void serviceDisplays() {
        if (!radioTxEnabled_) {
            startupPending_ = false;
            refreshMask_ = REFRESH_NONE;
            return;
        }

        if (startupPending_) {
            startupPending_ = false;

            if (!can_.sendStartup()) {
                startupPending_ = true;
                return;
            }

            can_.delayWithRx(60);
            requestRefresh(REFRESH_ALL);
        }

        // AUDIO
        if (refreshMask_ & REFRESH_AUDIO) {
            refreshMask_ &=
                (uint8_t)~REFRESH_AUDIO;

            const bool ok = pcHeartbeatAlive_
                ? audioPage_.show(
                    currentHeader_,
                    currentBody_
                )
                : audioPage_.show(
                    "RADIO",
                    "OFF"
                );

            if (!ok) {
                requestRefresh(REFRESH_AUDIO);
                return;
            }
        }

        // NAV przed TEL, bo TEL ma opcjonalne 786 ms
        // opóźnienia przed końcowym 7F.
        if (refreshMask_ & REFRESH_NAV) {
            refreshMask_ &=
                (uint8_t)~REFRESH_NAV;

            if (
                !navPage_.showReady(
                    pcHeartbeatAlive_
                )
            ) {
                requestRefresh(REFRESH_NAV);
                return;
            }
        }

        // TEL
        if (refreshMask_ & REFRESH_TEL) {
            refreshMask_ &=
                (uint8_t)~REFRESH_TEL;

            if (
                !telPage_.showReady(
                    pcHeartbeatAlive_
                )
            ) {
                requestRefresh(REFRESH_TEL);
            }
        }
    }

    // --------------------------------------------------
    // HEARTBEAT ANDROIDA
    // --------------------------------------------------

    void onHeartbeatArrived() {
        lastPcHeartbeatMs_ = millis();

        if (pcHeartbeatAlive_) {
            return;
        }

        pcHeartbeatAlive_ = true;

        strncpy(
            currentHeader_,
            "CONNECTED",
            sizeof(currentHeader_) - 1
        );
        currentHeader_[
            sizeof(currentHeader_) - 1
        ] = '\0';

        strncpy(
            currentBody_,
            "NO META",
            sizeof(currentBody_) - 1
        );
        currentBody_[
            sizeof(currentBody_) - 1
        ] = '\0';

        requestRefresh(REFRESH_ALL);
        sendStatus();
    }

    void onHeartbeatLost() {
        if (!pcHeartbeatAlive_) {
            return;
        }

        pcHeartbeatAlive_ = false;
        setAudioOffText();

        requestRefresh(REFRESH_ALL);
        sendStatus();
    }

    void checkPcHeartbeat() {
        if (
            pcHeartbeatAlive_ &&
            elapsed(
                lastPcHeartbeatMs_,
                PC_HEARTBEAT_TIMEOUT_MS
            )
        ) {
            onHeartbeatLost();
        }
    }

    // --------------------------------------------------
    // KLUCZYK
    // --------------------------------------------------

    static KeyState decodeKeyState(
        uint8_t firstByte
    ) {
        switch (firstByte) {
            case 0x00: return KEY_OUT;
            case 0x01: return KEY_INSERTED;
            case 0x03: return KEY_POS1;
            case 0x0F: return KEY_IGNITION;
            default:   return KEY_UNKNOWN;
        }
    }

    static bool shouldRadioRun(
        KeyState state
    ) {
        return (
            state == KEY_POS1 ||
            state == KEY_IGNITION
        );
    }

    void resetButtonDebounce() {
        buttonsArmAtMs_ =
            millis() + BUTTON_ARM_DELAY_MS;

        lastButtonEventMs_ = 0;
        lastTelButtonMs_ = 0;
        lastButtonKey_ = BTN_NONE;
        seenRealButtonSinceArm_ = false;
    }

    void radioStart() {
        if (radioTxEnabled_) {
            return;
        }

        radioTxEnabled_ = true;

        can_.setEnabled(true);
        can_.resetSessionSequence();

        clusterRequestedKeepalive_ = false;
        nextKeepaliveMs_ =
            millis() + KEEPALIVE_MS;

        resetButtonDebounce();

        startupPending_ = true;
        requestRefresh(REFRESH_ALL);
    }

    void radioStop() {
        if (!radioTxEnabled_) {
            return;
        }

        radioTxEnabled_ = false;
        can_.setEnabled(false);

        clusterRequestedKeepalive_ = false;
        nextKeepaliveMs_ = 0;

        buttonsArmAtMs_ = 0;
        lastButtonKey_ = BTN_NONE;
        seenRealButtonSinceArm_ = false;

        startupPending_ = false;
        refreshMask_ = REFRESH_NONE;
    }

    void parseKeyFrame(
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
            newState == KEY_UNKNOWN ||
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
            radioStart();
        } else if (wasRunning && !nowRunning) {
            radioStop();
        }

        sendStatus();
    }

    // --------------------------------------------------
    // PRZYCISKI
    // --------------------------------------------------

    bool buttonsAreArmed() const {
        return (
            radioTxEnabled_ &&
            pcHeartbeatAlive_ &&
            buttonsArmAtMs_ != 0 &&
            due(buttonsArmAtMs_)
        );
    }

    void sendButton(uint8_t button) {
        if (!pcHeartbeatAlive_) {
            return;
        }

        const uint8_t payload[1] = {
            button
        };

        protoSend(
            MSG_BUTTON,
            payload,
            sizeof(payload)
        );
    }

    void parseSteeringButtonFrame(
        const twai_message_t& frame
    ) {
        if (
            frame.identifier != 0x1D0 ||
            frame.data_length_code < 4
        ) {
            return;
        }

        // ACK Bx/9x nie mogą zostać uznane za przyciski.
        if (
            frame.data[0] != 0xAF ||
            frame.data[1] != 0x01
        ) {
            return;
        }

        if (!buttonsAreArmed()) {
            return;
        }

        const uint8_t key = frame.data[3];
        const uint32_t now = millis();

        if (
            !seenRealButtonSinceArm_ &&
            key == BTN_RELEASE
        ) {
            lastButtonKey_ = BTN_RELEASE;
            return;
        }

        if (key == lastButtonKey_) {
            return;
        }

        if (key == BTN_RELEASE) {
            lastButtonKey_ = BTN_RELEASE;
            sendButton(key);
            return;
        }

        if (
            lastButtonEventMs_ != 0 &&
            now - lastButtonEventMs_ <
                BUTTON_DEBOUNCE_MS
        ) {
            return;
        }

        lastButtonEventMs_ = now;
        lastButtonKey_ = key;
        seenRealButtonSinceArm_ = true;

        sendButton(key);
    }

    void parseTelButtonFrame(
        const twai_message_t& frame
    ) {
        if (
            frame.identifier != 0x1A8 ||
            frame.data_length_code < 2 ||
            !buttonsAreArmed()
        ) {
            return;
        }

        const uint8_t b0 = frame.data[0];
        const uint8_t b1 = frame.data[1];
        const uint32_t now = millis();

        if (
            lastTelButtonMs_ != 0 &&
            now - lastTelButtonMs_ <
                TEL_BUTTON_DEBOUNCE_MS
        ) {
            return;
        }

        if (b0 == 0x40 && b1 == 0x00) {
            lastTelButtonMs_ = now;
            sendButton(BTN_BRIGHTNESS_UP);
            return;
        }

        // Celowo tylko 0x80, żeby 0x10 nie kolidowało
        // z przyciskiem VOL+.
        if (b0 == 0x80 && b1 == 0x00) {
            lastTelButtonMs_ = now;
            sendButton(BTN_BRIGHTNESS_DOWN);
        }
    }

    // --------------------------------------------------
    // SERIAL TX
    // --------------------------------------------------

    void protoSend(
        uint8_t type,
        const uint8_t* payload,
        uint8_t payloadLength
    ) {
        if (
            payloadLength >
            PROTO_MAX_LEN - 2
        ) {
            return;
        }

        const uint8_t length =
            (uint8_t)(2 + payloadLength);

        uint8_t temporary[PROTO_MAX_LEN];

        temporary[0] =
            protoTxSequence_++;

        if (protoTxSequence_ == 0) {
            protoTxSequence_ = 1;
        }

        temporary[1] = type;

        for (
            uint8_t index = 0;
            index < payloadLength;
            index++
        ) {
            temporary[2 + index] =
                payload[index];
        }

        const uint8_t crc =
            crc8Calculate(
                length,
                temporary
            );

        Serial.write(SOF1);
        Serial.write(SOF2);
        Serial.write(length);
        Serial.write(temporary, length);
        Serial.write(crc);
        Serial.flush();
    }

    void protoAck(
        uint8_t acknowledgedSequence,
        uint8_t acknowledgedType,
        uint8_t status
    ) {
        const uint8_t payload[3] = {
            acknowledgedSequence,
            acknowledgedType,
            status
        };

        protoSend(
            MSG_ACK,
            payload,
            sizeof(payload)
        );
    }

    void sendStatus() {
        const uint8_t payload[3] = {
            (uint8_t)keyState_,
            (uint8_t)(radioTxEnabled_ ? 1 : 0),
            (uint8_t)(pcHeartbeatAlive_ ? 1 : 0)
        };

        protoSend(
            MSG_STATUS,
            payload,
            sizeof(payload)
        );
    }

    // --------------------------------------------------
    // SERIAL RX
    // --------------------------------------------------

    static void copyLimitedText(
        char* destination,
        size_t destinationSize,
        const uint8_t* source,
        uint8_t sourceLength,
        const char* fallback
    ) {
        if (destinationSize == 0) {
            return;
        }

        if (sourceLength == 0) {
            strncpy(
                destination,
                fallback,
                destinationSize - 1
            );

            destination[
                destinationSize - 1
            ] = '\0';

            return;
        }

        uint8_t length = sourceLength;

        if (length > destinationSize - 1) {
            length =
                (uint8_t)(destinationSize - 1);
        }

        for (
            uint8_t index = 0;
            index < length;
            index++
        ) {
            char character =
                (char)source[index];

            if (
                (uint8_t)character < 0x20 ||
                (uint8_t)character > 0x7E
            ) {
                character = ' ';
            }

            destination[index] =
                character;
        }

        destination[length] = '\0';
    }

    uint8_t handleHeartbeat(
        const uint8_t* payload,
        uint8_t payloadLength
    ) {
        (void)payload;

        if (payloadLength != 0) {
            return ACK_BAD_PAYLOAD;
        }

        onHeartbeatArrived();
        return ACK_OK;
    }

    uint8_t handleSetText(
        const uint8_t* payload,
        uint8_t payloadLength
    ) {
        if (!pcHeartbeatAlive_) {
            return ACK_RADIO_OFF;
        }

        if (payloadLength < 2) {
            return ACK_BAD_PAYLOAD;
        }

        const uint8_t headerLength =
            payload[0];

        const uint8_t bodyLength =
            payload[1];

        if (
            (uint16_t)2 +
            headerLength +
            bodyLength != payloadLength
        ) {
            return ACK_BAD_PAYLOAD;
        }

        const uint8_t* header =
            payload + 2;

        const uint8_t* body =
            payload + 2 + headerLength;

        copyLimitedText(
            currentHeader_,
            sizeof(currentHeader_),
            header,
            headerLength,
            "CONNECTED"
        );

        copyLimitedText(
            currentBody_,
            sizeof(currentBody_),
            body,
            bodyLength,
            "NO META"
        );

        // MSG_SET_TEXT zmienia na razie tylko AUDIO.
        requestRefresh(REFRESH_AUDIO);

        return ACK_OK;
    }

    void handleProtocolPacket(
        uint8_t sequence,
        uint8_t type,
        const uint8_t* payload,
        uint8_t payloadLength
    ) {
        if (type == MSG_ACK) {
            return;
        }

        uint8_t status =
            ACK_UNKNOWN_TYPE;

        switch (type) {
            case MSG_HEARTBEAT:
                status = handleHeartbeat(
                    payload,
                    payloadLength
                );
                break;

            case MSG_SET_TEXT:
                status = handleSetText(
                    payload,
                    payloadLength
                );
                break;

            default:
                status = ACK_UNKNOWN_TYPE;
                break;
        }

        protoAck(
            sequence,
            type,
            status
        );

        if (
            type == MSG_SET_TEXT &&
            status == ACK_OK
        ) {
            sendStatus();
        }
    }

    void readSerialProtocol() {
        while (Serial.available()) {
            const uint8_t byte =
                (uint8_t)Serial.read();

            switch (serialState_) {
                case 0:
                    if (byte == SOF1) {
                        serialState_ = 1;
                    }
                    break;

                case 1:
                    if (byte == SOF2) {
                        serialState_ = 2;
                    } else if (byte == SOF1) {
                        serialState_ = 1;
                    } else {
                        serialState_ = 0;
                    }
                    break;

                case 2:
                    serialLength_ = byte;

                    if (
                        serialLength_ < 2 ||
                        serialLength_ >
                            PROTO_MAX_LEN
                    ) {
                        serialState_ = 0;
                        break;
                    }

                    serialIndex_ = 0;
                    serialState_ = 3;
                    break;

                case 3:
                    serialBuffer_[
                        serialIndex_++
                    ] = byte;

                    if (
                        serialIndex_ >=
                        serialLength_ + 1
                    ) {
                        const uint8_t receivedCrc =
                            serialBuffer_[
                                serialLength_
                            ];

                        const uint8_t calculatedCrc =
                            crc8Calculate(
                                serialLength_,
                                serialBuffer_
                            );

                        if (
                            receivedCrc ==
                            calculatedCrc
                        ) {
                            const uint8_t sequence =
                                serialBuffer_[0];

                            const uint8_t type =
                                serialBuffer_[1];

                            const uint8_t* payload =
                                serialBuffer_ + 2;

                            const uint8_t payloadLength =
                                (uint8_t)(
                                    serialLength_ - 2
                                );

                            handleProtocolPacket(
                                sequence,
                                type,
                                payload,
                                payloadLength
                            );
                        }

                        serialState_ = 0;
                    }
                    break;

                default:
                    serialState_ = 0;
                    break;
            }
        }
    }

    // --------------------------------------------------
    // UŚPIENIE
    // --------------------------------------------------

    void checkCanSleep() {
        if (
            !elapsed(
                bootMs_,
                STARTUP_SLEEP_ARM_MS
            )
        ) {
            return;
        }

        if (keyState_ != KEY_OUT) {
            return;
        }

        if (
            elapsed(
                lastCanActivityMs_,
                CAN_IDLE_SLEEP_MS
            )
        ) {
            powerOffEsp();
        }
    }
};

// ======================================================
// PROGRAM
// ======================================================

VehicleDisplayController controller;

void setup() {
    controller.begin();
}

void loop() {
    controller.service();
}
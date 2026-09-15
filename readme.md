# Mercedes W209 – ESP32 ↔ Instrument Cluster (CAN)  

Szczegółowy opis działania firmware, przypisania pinów oraz używanych protokołów komunikacji dla projektu `Mercedes_Can_Bus`.

## 1. Cel projektu

ESP32-C3 pełni rolę pośrednika między:
- **Android/PC** (UART/USB CDC, własny protokół ramek),
- **licznikiem/instrument cluster** (CAN przez transceiver TJA1053T).

Firmware:
- utrzymuje sesję z licznikiem (startup + keepalive),
- wysyła strony **AUDIO / TEL / NAV** do wyświetlacza licznika,
- odbiera przyciski z kierownicy i telefonu, a następnie przekazuje je do Androida,
- zarządza zasilaniem i usypianiem.

## 2. Pinout (ESP32-C3 + TJA1053T)

Definicje znajdują się w `include/Config.h`.

| Sygnał | GPIO | Funkcja |
|---|---:|---|
| `CAN_TXD_PIN` | 3 | TX kontrolera TWAI (CAN) do transceivera |
| `CAN_RXD_PIN` | 2 | RX kontrolera TWAI (CAN) z transceivera |
| `BUCK_ENABLE_PIN` | 0 | Włączanie/odcinanie przetwornicy zasilającej |
| `TJA_STB_PIN` | 6 | Linia STB transceivera TJA1053T |
| `TJA_EN_PIN` | 7 | Linia EN transceivera TJA1053T |
| `TOP_AMBIENT` | 21 | PWM podświetlenia (ustawiane w `src/main.cpp`) |

### Tryby transceivera (z `src/PowerManager.cpp`)

- **Normal mode:** `EN=HIGH`, `STB=HIGH`
- **Go-to-sleep:** `EN=HIGH`, `STB=LOW` (krótki impuls)
- **Sleep:** `EN=LOW`, `STB=LOW`

## 3. Architektura firmware

Główne klasy:

- `VehicleController` – logika nadrzędna stanu pojazdu/radia i orchestracja.
- `CanTransport` – warstwa transportowa CAN (TWAI, segmentacja, ACK/retry).
- `AndroidProtocol` – parser/enkoder własnego protokołu UART.
- `AudioPage`, `TelephonePage`, `NavigationPage` – budowa i wysyłanie payloadów do licznika.
- `PowerManager` – inicjalizacja i odcięcie zasilania.
- `TextEncoding` – normalizacja znaków (ASCII, uppercase, filtrowanie).

Pętla główna (`src/main.cpp`):
- `setup()` → `controller.begin()`
- `loop()` → `controller.service()`

## 4. Protokół CAN do licznika (warstwa transportowa)

### ID CAN

Z `include/Config.h`:
- `RADIO_TO_CLUSTER_ID = 0x1A4` (ESP/radio → licznik)
- `CLUSTER_TO_RADIO_ID = 0x1D0` (licznik → ESP/radio)

### Start i podtrzymanie sesji

`CanTransport`:
- **startup:** `A0 01 00 00 00 00 00 00`
- **keepalive:** `A1 01 00 00 00 00 00 00`

`VehicleController`:
- keepalive cykliczny: `KEEPALIVE_MS = 1000`
- dodatkowo keepalive na żądanie licznika (`0x1D0`, bajt0=`0xA3`)

### Segmentacja danych logicznych

Payload logiczny musi mieć długość wielokrotności 7 bajtów.

Segment CAN (8 bajtów):
- `frame[0]` = nibble sekwencji (0..15),
- bit `0x10` ustawiany w ostatnim segmencie,
- `frame[1..7]` = 7 bajtów danych.

### ACK / retry

Po każdym segmencie oczekiwane są ACK z `0x1D0`:
- **Accepted:** `0xB0 | ((seq + 1) & 0x0F)`
- **Retry:** `0x90 | (seq & 0x0F)`

Parametry:
- `ACK_TIMEOUT_MS = 60`
- `ACK_RETRY_DELAY_MS = 115`
- `MAX_SEGMENT_ATTEMPTS = 5`

## 5. Protokół UART ESP ↔ Android

Format ramki (`AndroidProtocol`):
- `SOF1 = 0xAA`
- `SOF2 = 0x55`
- `LEN` (1 bajt)
- `DATA[LEN]` gdzie `DATA = [SEQ, TYPE, PAYLOAD...]`
- `CRC8` (polinom `0x07`, liczony od `LEN` i `DATA`)

### Typy wiadomości

Android → ESP:
- `0x11` heartbeat
- `0x12` set text (AUDIO)
- `0x13` set telephone

ESP → Android:
- `0x20` button (`[page, button]`)
- `0x21` status (`[keyState, radioEnabled, heartbeatAlive]`)
- `0x7F` ACK (`[ackedSeq, ackedType, status]`)

Status ACK:
- `0x00` OK
- `0x01` bad payload
- `0x02` unknown type
- `0x03` radio off

## 6. Strony wyświetlacza i payloady

Stałe payloadów są w `include/DisplayPayloads.h`.

### AUDIO
- Header opcode `0x11`, body opcode `0x12`.
- Tekst kodowany/normalizowany przez `TextEncoding`.
- Długości:
  - header: 28 bajtów (max tekst 11 znaków),
  - body: 21 bajtów (max tekst 10 znaków).

### TEL
- Control: `0x7F 0x01 ...`
- Header: `0x71 0x01 'T' 'E' 'L' ...`
- Body: `0x72 0x01 ...`
- Format dynamiczny: `72 01 + BODY1[5] + BODY2 + 0D + BODY3 + 00`, potem padding do wielokrotności 7.

### NAV
- Control: `0x6F 0x01 ...`
- Header: `0x61 0x01 'N' 'A' 'V' ...`
- Body: `0x62 0x01 ...`

## 7. Wejścia z pojazdu i mapowanie przycisków

### Stan kluczyka (`ID 0x000`, bajt0)
- `0x00` Out
- `0x01` Inserted
- `0x03` Position1
- `0x0F` Ignition

Radio aktywuje się dla `Position1` i `Ignition`.

### Przyciski z kierownicy (`ID 0x1D0`)

Ramka przycisku:
- bajt0=`0xAF`, bajt1=`0x01`
- bajt2=`activePage`
- bajt3=`buttonCode`

Obsługa debounce, hold-repeat oraz release jest realizowana w `VehicleController::parseSteeringButton`.

### Przyciski telefonu (`ID 0x1A8`)
- `0x40 0x00` – pickup / jasność+
- `0x80 0x00` – hangup / jasność-

## 8. Odświeżanie treści na liczniku

`VehicleController` utrzymuje maskę odświeżania i wysyła jedną stronę na iterację pętli.

Aktualne timery:
- `DISPLAY_REFRESH_MS = 1500` – okresowe ponowne wysłanie aktywnej strony,
- `KEEPALIVE_MS = 1000` – podtrzymanie sesji.

Celem okresowego refreshu jest ograniczenie ryzyka „pustego” wyświetlacza licznika.

## 9. Zarządzanie energią

Po bezczynności CAN (`CAN_IDLE_SLEEP_MS`) i przy kluczyku w stanie `Out`:
- zatrzymywana jest transmisja,
- transceiver przechodzi do sleep,
- odcinane jest zasilanie przez `BUCK_ENABLE_PIN`.

## 10. Build / środowisko

`platformio.ini`:
- platform: `espressif32`
- board: `esp32-c3-devkitm-1`
- framework: `arduino`
- monitor: `115200`

## 11. Znane ograniczenia

- Trigger „incoming call niezależnie od aktywnej strony” nie jest jeszcze zdekodowany z CAN.
- Limity tekstu TEL po stronie wejścia i transportu mogą wymagać dalszego dopasowania pod realne przypadki.


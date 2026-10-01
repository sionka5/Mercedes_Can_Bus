# Mercedes W209 CAN Bridge (ESP32-C3)

This project turns an ESP32-C3 into a small bridge between an Android host and the Mercedes W209 instrument cluster.

On one side, the ESP32 receives custom UART packets (USB CDC / serial). On the other side, it talks to the cluster over CAN through a TJA1053T transceiver and emulates the radio-side display protocol.

## What the firmware does

- starts and maintains the radio/cluster CAN session,
- renders AUDIO / TEL / NAV pages on the cluster display,
- forwards steering wheel and phone button events to Android,
- publishes telemetry back to Android (RPM, speed, ambient PWM),
- handles inactivity shutdown and transceiver sleep.

## Hardware mapping

Defined in `/home/runner/work/Mercedes_Can_Bus/Mercedes_Can_Bus/include/Config.h`.

| Signal | GPIO | Purpose |
|---|---:|---|
| `CAN_TXD_PIN` | 3 | ESP32 TWAI TX to transceiver |
| `CAN_RXD_PIN` | 2 | ESP32 TWAI RX from transceiver |
| `BUCK_ENABLE_PIN` | 0 | Main power converter enable/cut |
| `TJA_STB_PIN` | 6 | TJA1053T standby control |
| `TJA_EN_PIN` | 7 | TJA1053T enable control |
| `TOP_AMBIENT` | 21 | Ambient/backlight PWM output |

Transceiver states used by `PowerManager`:

- **Normal**: `EN=HIGH`, `STB=HIGH`
- **Go-to-sleep pulse**: `EN=HIGH`, `STB=LOW`
- **Sleep**: `EN=LOW`, `STB=LOW`

## Project architecture

Main components:

- `VehicleController`: central state machine (ignition state, heartbeat, page refresh, buttons, telemetry, sleep).
- `CanTransport`: TWAI driver wrapper, CAN TX/RX, segmented payload transport, ACK/retry handling.
- `AndroidProtocol`: custom UART protocol parser/encoder with CRC8 and per-message ACK.
- `AudioPage`, `TelephonePage`, `NavigationPage`: build page payloads and send them through `CanTransport`.
- `PowerManager`: transceiver mode control and hard power cut path.
- `Watchdog`: task watchdog init/feed wrapper.

Execution flow:

- `setup()` -> `controller.begin()`
- `loop()` -> `controller.service()`

## CAN transport details

### CAN IDs

From `Config.h`:

- `RADIO_TO_CLUSTER_ID = 0x1A4` (ESP/radio emulation -> cluster)
- `CLUSTER_TO_RADIO_ID = 0x1D0` (cluster -> ESP/radio emulation)

Other consumed frames in `VehicleController`:

- `0x000`: key state
- `0x1A8`: phone/brightness buttons
- `0x002`: RPM
- `0x003` and `0x10A`: speed
- `0x00C`: ambient brightness source

### Session handshake

Session startup in `CanTransport::sendStartup()`:

1. reset segment sequence nibble,
2. send startup frame:
   - `A0 01 00 00 00 00 00 00`
3. short delay with RX handling,
4. send keepalive frame:
   - `A1 01 00 00 00 00 00 00`

Keepalive is also sent:

- periodically every `KEEPALIVE_MS` (1000 ms),
- immediately when cluster requests it via `0x1D0` with first byte `0xA3`.

### Logical packet segmentation

Cluster payloads are sent as logical packets split into 8-byte CAN frames.

Frame format:

- `frame[0]`: sequence nibble (`0..15`),
- `frame[0]` bit `0x10`: end-of-logical-packet marker,
- `frame[1..7]`: payload bytes.

Constraint:

- logical payload length must be divisible by 7.

### ACK / retry contract

For each sent segment, cluster is expected to return ACK on `0x1D0`:

- accepted: `0xB0 | ((seq + 1) & 0x0F)`
- retry: `0x90 | (seq & 0x0F)`

Timing and limits:

- `ACK_TIMEOUT_MS = 60`
- `ACK_RETRY_DELAY_MS = 115`
- `MAX_SEGMENT_ATTEMPTS = 5`

If a segment fails, the same sequence number is retried. Sequence increments only after accepted ACK.

## Display protocol pages

Static payload templates are in `/home/runner/work/Mercedes_Can_Bus/Mercedes_Can_Bus/include/DisplayPayloads.h`.

### AUDIO page

- Header opcode: `0x11`
- Body opcode: `0x12`
- Header payload length: 28 bytes (text area starts at offset 2)
- Body payload length: 21 bytes (text area starts at offset 7)

Flow:

1. send audio header,
2. wait `AUDIO_HEADER_BODY_GAP_MS` (80 ms) with RX processing,
3. send audio body.

### TEL page

Packets sent in sequence:

1. control (`0x7F 0x01 ...`),
2. header (`0x71 0x01 'T' 'E' 'L' ...`),
3. dynamic body.

Dynamic body format:

- `72 01 + BODY1[5] + BODY2 + 0D + BODY3 + 00 + padding to multiple of 7`

Notes:

- BODY1 is right-aligned in fixed 5 characters,
- BODY2 and BODY3 are bounded to 30 characters each,
- optional closing control is disabled by default (`TEL_SEND_CLOSING_CONTROL = false`) because it may close newer screens during fast transitions.

### NAV page

Packets sent in sequence:

1. control (`0x6F 0x01 ...`),
2. header (`0x61 0x01 'N' 'A' 'V' ...`),
3. body (`0x62 0x01 ...`) with READY/OFF state.

Final closing control is disabled by default (`NAV_SEND_CLOSING_CONTROL = false`) because it was observed clearing NAV content.

## UART protocol (ESP32 <-> Android)

### Packet format

`AndroidProtocol` frame:

- `SOF1 = 0xAA`
- `SOF2 = 0x55`
- `LEN` (1 byte)
- `DATA[LEN]`, where DATA = `[SEQ, TYPE, PAYLOAD...]`
- `CRC8`

CRC8:

- polynomial `0x07`
- computed over `LEN` and full `DATA`

### Message types

Android -> ESP:

- `0x11`: heartbeat
- `0x12`: set audio text (`header`, `body`)
- `0x13`: set telephone text (`body1`, `body2`, `body3`)
- `0x05`: set ambient max percent (`0..100`)

ESP -> Android:

- `0x20`: button event `[page, button]`
- `0x21`: status `[keyState, radioEnabled, heartbeatAlive]`
- `0x04`: telemetry `[rpm(2), speed(float32), currentPwm, maxPercent]`
- `0x7F`: protocol ACK `[ackedSeq, ackedType, status]`

ACK statuses:

- `0x00`: OK
- `0x01`: bad payload
- `0x02`: unknown type
- `0x03`: radio off / heartbeat missing

## Vehicle inputs and button mapping

### Key state (`CAN ID 0x000`, `data[0]`)

- `0x00` -> Out
- `0x01` -> Inserted
- `0x03` -> Position1
- `0x0F` -> Ignition

Radio is enabled only in `Position1` or `Ignition`.

### Steering wheel buttons (`CAN ID 0x1D0`)

Expected packet marker:

- `data[0] = 0xAF`
- `data[1] = 0x01`
- `data[2] = activePage`
- `data[3] = buttonCode`

Handled behavior:

- arm delay after radio start,
- debounce,
- long-press repeat for volume buttons,
- explicit release event forwarding.

### Phone / brightness buttons (`CAN ID 0x1A8`)

- `40 00` -> phone pickup on TEL page, otherwise brightness up
- `80 00` -> phone hangup on TEL page, otherwise brightness down

## Refresh and timing strategy

Runtime timers from `Config.h`:

- `KEEPALIVE_MS = 1000`
- `DISPLAY_REFRESH_MS = 1500`
- `FRAME_GAP_MS = 8`
- `PAGE_PACKET_GAP_MS = 8`
- `BUTTON_DEBOUNCE_MS = 300`
- `BUTTON_HOLD_DELAY_MS = 450`
- `BUTTON_REPEAT_MS = 120`

`VehicleController` keeps a refresh bitmask and sends one page path per loop iteration to avoid flooding the bus.

## Power management and sleep

When all conditions are true:

- startup guard elapsed (`STARTUP_SLEEP_ARM_MS`),
- key state is `Out`,
- no CAN activity for `CAN_IDLE_SLEEP_MS`,

controller calls `shutdown()`:

- disables radio TX,
- stops TWAI driver,
- requests transceiver sleep,
- cuts main power via `BUCK_ENABLE_PIN`.

## Watchdog notes and bug fix

Watchdog wrapper supports ESP-IDF task watchdog when available (`esp_task_wdt`).

- enable flag: `ENABLE_TASK_WATCHDOG`
- timeout: `WATCHDOG_TIMEOUT_MS`
- default in this repo: disabled (`false`) for bring-up safety

Potential issue found in ACK wait loop: when the bus was busy with non-ACK frames, watchdog feeding happened mainly on timeout path. The loop now feeds watchdog also after handling non-ACK received frames, which makes behavior more robust under heavy CAN traffic.

## Build / environment

From `/home/runner/work/Mercedes_Can_Bus/Mercedes_Can_Bus/platformio.ini`:

- platform: `espressif32`
- board: `esp32-c3-devkitm-1`
- framework: `arduino`
- monitor speed: `115200`

## Current limitations

- Incoming-call trigger independent of active page is still not decoded from CAN.
- TEL text limits and behavior may still need tuning on real hardware edge cases.

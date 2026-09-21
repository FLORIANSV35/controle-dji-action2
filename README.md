# DJI Action 2 BLE Control — ESP32-C3

ESP32-C3 firmware that starts / stops recording on a DJI Action 2 over BLE when the Betaflight flight controller is armed / disarmed (detected over MSP).

- BLE connection (service 0xFFF0), DUML commands, camera state confirmation
- GATT cache prewarming for a fast first arming
- Goggles OSD through an ELRS Backpack (ESP-NOW): `C OK`, `C REC`, `C STP`, `ERR`

## Configuration
- `BIND_PHRASE`: **replace the dummy phrase with your own ELRS bind phrase**.
- Betaflight FC: RX = GPIO20, TX = GPIO21, 115200 baud.
- Testing without an FC: send `a` (arm) / `d` (disarm) on the serial port.

## Build
Arduino IDE or `arduino-cli`, board `esp32:esp32:esp32c3`, **NimBLE-Arduino** library (h2zero) 2.x.

---

## Français

Firmware ESP32-C3 qui démarre / arrête l'enregistrement d'une DJI Action 2 en BLE quand le contrôleur de vol Betaflight est armé / désarmé (détection via MSP).

- Connexion BLE (service 0xFFF0), commandes DUML, confirmation de l'état de la caméra
- Préchauffage du cache GATT pour un premier armement rapide
- OSD sur les goggles via un Backpack ELRS (ESP-NOW) : `C OK`, `C REC`, `C STP`, `ERR`

## Configuration
- `BIND_PHRASE` : **remplace la phrase bidon par ta propre bind phrase ELRS**.
- FC Betaflight : RX = GPIO20, TX = GPIO21, 115200 baud.
- Test sans FC : envoyer `a` (arm) / `d` (disarm) sur le port série.

## Compilation
Arduino IDE ou `arduino-cli`, carte `esp32:esp32:esp32c3`, lib **NimBLE-Arduino** (h2zero) 2.x.

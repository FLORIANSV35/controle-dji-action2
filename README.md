# Contrôle DJI Action 2 en BLE — ESP32-C3

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

# Architecture

## Goal

Keep the sensing system small and inexpensive without allowing the current Pico/USB prototype to become an accidental permanent dependency.

## Layer model

```text
SENSORS
  LSM6DSOX     LIS3MDL      sparse ToF
      \            |            /
       SENSOR-ACQUISITION LAYER
        raw samples + timestamps
                  |
          PACKET / RECORD MODEL
       versioned, language-neutral
                  |
          TRANSPORT ADAPTER
     USB now | BLE/Wi-Fi later
                  |
       HOST CAPTURE / REPLAY API
          PC Python | Android
                  |
        GEOMETRY / ESTIMATION CORE
 orientation -> rays -> pose -> map
                  |
        VISUALISATION / APPLICATIONS
```

## Portability boundary

The packet/record model is the key contract. Sensor hardware can change below it; PC/Android applications can change above it. Neither side should need to know that the other currently uses a Pico 2 W or USB.

### Firmware responsibilities

- configure sensors;
- preserve raw values and sensor-native timestamps;
- maintain MCU monotonic time;
- emit explicit sensor<->MCU clock observations;
- buffer records safely;
- frame/version records;
- expose transport health and drop/error counters.

### Firmware non-responsibilities for now

- SLAM;
- point-cloud registration;
- dense reconstruction;
- host UI;
- irreversible filtering that prevents later reprocessing.

### Host responsibilities

- validate framing/sequence/timestamps;
- losslessly record the live stream;
- replay recordings through the same interfaces;
- calibrate/project depth zones into 3D rays/points;
- estimate orientation/pose/map with explicit uncertainty;
- provide a reference implementation that Kotlin/Android can reproduce.

## Time model

Do not define time as `sample_index / nominal_rate`. The reference LSM6DSOX is nominally configured at 104 Hz but measured around 101 Hz because of its unit-specific oscillator trim. A replacement unit may differ.

Every IMU observation is tied to the LSM timestamp domain. Periodic direct timestamp reads correlate that domain to the MCU monotonic clock. Other sensors/host observations can then be mapped to a common timeline.

## Current vs future transports

- **Current:** Pico USB connection to PC while developing the record format and algorithms.
- **Early portability target:** Android USB host/OTG.
- **Later:** BLE, Wi-Fi, UART or local storage on a compact MCU such as an ESP32-class board.

Transport changes must not change record semantics.

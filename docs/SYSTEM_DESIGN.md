# System design

## 1. Functional overview

The electronic HTS lock moves the mechanism between OPEN and CLOSED positions using a DC motor controlled through an H-bridge. The rider requests a toggle using a handlebar push-button. The controller uses the wheel-speed signal and the derived longitudinal acceleration to determine whether locking is permitted.

The safe release direction is OPEN. If a safety threshold is reached while the mechanism is CLOSED or moving, the controller overrides rider input and commands release.

## 2. Requirements implemented

| ID | Requirement | Implementation |
|---|---|---|
| R1 | One toggle per button press | Debounced edge-triggered button input |
| R2 | HTS OPEN at speed ≥ 7 km/h | Automatic release interlock |
| R3 | HTS OPEN at acceleration ≥ 3 m/s² | Automatic release interlock |
| R4 | Ignore rider commands while interlock active | State-machine priority |
| R5 | Complete movement within 5 s | Movement timeout and latched FAULT |
| R6 | Detect both limit switches active | Conflict detection and FAULT |
| R7 | Never drive both motor directions simultaneously | H-bridge command logic and 50 ms reversal dead time |
| R8 | Provide rider feedback | Buzzer patterns |
| R9 | Removable service units | Keyed connectors and labelled harness |
| R10 | Observable operation | Event logging and 5 Hz telemetry |

## 3. Hardware

- Arduino Mega 2560, 5 V logic.
- 12 V vehicle supply through ignition-key switched power and a 15 A fuse.
- DC motor and H-bridge driver.
- Two limit switches for OPEN and CLOSED end positions.
- Hall-effect wheel sensor reading a 50-tooth phonic wheel.
- Signal conditioning path to analog input A10.
- Handlebar push-button, 16×2 LCD and buzzer.
- Modular signal-conditioning / input-expansion electronics.

The dossier specifies a wheel radius of 0.26 m and 32.7 mm of travel per phonic-wheel tooth.

## 4. Firmware logic

The main loop is non-blocking and is structured around sensor sampling, input updates, state-machine evaluation, motor control, buzzer scheduling and telemetry.

### States

- `IDLE`: no active movement.
- `MOVING`: normal rider-requested movement.
- `RELEASING`: safety-priority movement to OPEN.
- `FAULT`: motor disabled; fault latched until power-cycle.

### Interlock

```text
lockAllowed = (speed < 7 km/h) AND (acceleration < 3 m/s²)

if lockAllowed:
    accept one rider toggle per button press
else:
    ignore rider command
    if HTS is not OPEN:
        command release to OPEN
```

### Timing and signal conditioning

| Parameter | Value |
|---|---:|
| Speed / acceleration update | 100 ms |
| Acceleration calculation interval | 200 ms |
| Telemetry | 200 ms (5 Hz) |
| Movement timeout | 5 s |
| Limit-switch debounce | 5 ms |
| Push-button debounce | 30 ms |
| H-bridge reversal dead time | 50 ms |
| Impossible switch-state detection | 20 ms |
| No-edge timeout | 250 ms |
| Minimum accepted tooth spacing | 500 µs |
| Comparator hysteresis thresholds | 1.9 V rising / 1.7 V falling |

## 5. Wiring and serviceability

The harness uses a main trunk along the vehicle frame with branches routed to each component. Connector, terminal and wire-gauge choices are defined per circuit. The harness was assembled on a board from the wiring drawings, labelled and then routed on the vehicle. Keyed automotive connectors allow individual control units to be removed for debugging and service.

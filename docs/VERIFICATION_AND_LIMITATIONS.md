# Verification, validation and engineering limitations

## Evidence available

The project dossier describes:
- Bench debugging before vehicle installation.
- Two prototypes built.
- Road-testing activity at QJ Motor in China.
- Firmware v2.0 compiled unchanged on a PC and exercised against simulated wheel-sensor, limit-switch, motor and mechanism models.

The simulation validates firmware logic and timing under the model assumptions. It does not establish hardware reliability, vehicle-level safety, homologation or functional-safety compliance.

## Simulation results

| Test | Scenario | Result |
|---|---|---|
| T1 | Constant speed: 1, 3, 5, 7, 20, 50 km/h | Speed estimation error below 0.05% in the tested cases; acceleration noise ±0.04 m/s² up to 7 km/h |
| T2 | Rider toggles CLOSED → OPEN → CLOSED at standstill | Both requested movements completed in the model |
| T3 | Acceleration 0 → 10 km/h at 0.77 m/s² | Release started at measured 7.06 km/h; OPEN reached 1.47 s later |
| T4 | Acceleration threshold sweep | No release at 1.5 / 2.5 m/s²; release at 3.5 / 4.5 m/s² |
| T5 | Mechanism jammed | FAULT after 5.031 s; motor outputs LOW |
| T6 | Both limit switches active | FAULT after 25 ms; motor outputs LOW |
| T7 | Power-up mid-travel | Mechanism returned to OPEN in 0.721 s |
| T8 | Button held for 10 s | One movement only |
| T9 | One-sample interference spike every 0.5 s at 5 km/h | No spurious release in the simulated scenario |

The simulation also reports zero events in which both H-bridge inputs were HIGH.

## Known open points from the design review

These are explicitly identified as further verification or design actions, not completed features:

1. **Wheel-sensor plausibility:** a failed, open or shorted sensor could result in a zero-speed estimate. Add signal-range plausibility checks and consider cross-checking with an independent speed source.
2. **Signal interference:** validate the actual signal levels with an oscilloscope; assess an RC filter and shielded/twisted wiring.
3. **Limit-switch wiring faults:** consider normally-closed contacts and validate broken-wire detection.
4. **Motor stall:** add current measurement to detect a stalled actuator faster than the 5 s movement timeout.
5. **Controller reset:** verify H-bridge input states during reset and consider pull-down resistors and a hardware watchdog.
6. **Power loss:** define the mechanical fail-safe position if supply is lost during movement.
7. **Fault while not OPEN:** define the vehicle-level strategy if a fault occurs while HTS is not in the safe position.

## Important distinction

The broader system concept includes IMU-based tilt estimation, rider display and additional vehicle signals. The v2.0 firmware documented here implements the wheel-speed acquisition, acceleration calculation, OPEN/CLOSED interlock, actuator drive, fault handling and telemetry. Do not interpret the broader concept description as proof that all those additional functions are implemented in this firmware.

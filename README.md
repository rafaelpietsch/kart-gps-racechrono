# kart-gps

GNSS and inertial telemetry for [RaceChrono](https://racechrono.com), running on an
ESP32-C3 Super Mini. A GY-NEO6MV2 supplies position, an MPU-6050 supplies
acceleration and rotation, and both are published over Bluetooth LE using
RaceChrono's DIY device API so the phone logs them like any commercial receiver.

Built for kart track days, where the corners are short, the kerbs are violent and
a commercial logger costs more than a set of tyres.

![status](https://img.shields.io/badge/tests-127%20passing-brightgreen)
![license](https://img.shields.io/badge/license-MIT-blue)
![platform](https://img.shields.io/badge/platform-ESP32--C3-orange)

---

## What it does

| Channel | Rate | Source |
| --- | --- | --- |
| Position, speed, bearing, altitude, DOP | 5 Hz | GY-NEO6MV2 (u-blox NEO-6M) |
| 3-axis acceleration and 3-axis rotation | 25 Hz | MPU-6050 |
| Session clock, fix quality, device status | 1 Hz | firmware |

A single button zeroes the inertial sensors, restarts the session clock and, when
held, hot restarts the GNSS receiver.

## About the 25 Hz target

**The motion channel runs at 25 Hz. The GNSS channel cannot: the NEO-6M's
navigation engine tops out at 5 Hz**, which is a limit of the u-blox 6 silicon,
not of this firmware or of the link. Sending it a 25 Hz `CFG-RATE` does not make
it produce 25 solutions per second; it either rejects the message or reports the
same solution repeatedly.

This is normally the right trade anyway. GNSS position noise dominates over the
5 ms that separate consecutive 25 Hz samples, so the extra epochs would carry
almost no new information, while the inertial channel genuinely does resolve
what happens between them: brake application, kerb strikes, slides, throttle
pickup. RaceChrono interpolates the position track and overlays the 25 Hz motion
data on top of it.

If you want a genuinely faster position channel, the receiver has to change. The
firmware already parameterises the rate:

| Receiver | Max navigation rate | Change needed |
| --- | --- | --- |
| NEO-6M (this build) | 5 Hz | none, this is the default |
| NEO-M8N | 10 Hz | `-DKARTGPS_GPS_RATE_HZ=10` |
| NEO-M9N / M10 | 25 Hz | `-DKARTGPS_GPS_RATE_HZ=25`, and raise the UART baud |

Everything else — the parser, the encoders, the pipeline — already handles those
rates and is tested at them.

## Hardware

| Signal | ESP32-C3 pin | Connects to |
| --- | --- | --- |
| GPS TX -> ESP RX | GPIO20 | NEO-6M `TX` |
| ESP TX -> GPS RX | GPIO21 | NEO-6M `RX` |
| I2C SDA | GPIO5 | MPU-6050 `SDA` |
| I2C SCL | GPIO6 | MPU-6050 `SCL` |
| SET button | GPIO3 | button to GND |
| Status LED | GPIO8 | on-board LED |

Both modules run from the board's 3V3 rail. Full wiring notes, current draw and
the reasoning behind the pin choices are in [docs/hardware.md](docs/hardware.md).

## Build and flash

```bash
pio run -e esp32c3_supermini -t upload
```

```bash
pio device monitor
```

The debug environment adds verbose logging and echoes raw NMEA:

```bash
pio run -e esp32c3_supermini_debug -t upload
```

## Tests

The signal processing, protocol encoding and scheduling logic is hardware
independent and runs on the host:

```bash
pio test -e native
```

127 test cases across six suites. Two of them are worth calling out:

- `test_rc_protocol` checks the 20-byte GPS packet against a golden vector
  worked out by hand from the RaceChrono specification, so a layout or
  endianness mistake fails the build rather than producing plausible garbage.
- `test_ublox` checks every generated UBX frame against the byte sequences
  published by the [bonogps](https://github.com/renatobo/bonogps) project — an
  independent implementation, which makes it a real cross-check rather than a
  tautology.

`test_integration` drives the whole chain, from recorded NMEA bytes and
synthetic MPU-6050 register reads through to the exact notifications that would
have gone over the air.

## Using it with RaceChrono

1. Add a new device: **Settings -> Other devices -> Add device -> Other
   Bluetooth LE device**, and pick `KartGPS`.
2. Enable it as a GPS source.
3. Import the channel definitions for the motion and status data, listed in
   [docs/racechrono-setup.md](docs/racechrono-setup.md).

## How it is put together

```
src/                 ESP32 only: NimBLE server, I2C and UART drivers, main loop
lib/telemetry/       shared domain types
lib/nmea/            sentence assembly and fix parsing
lib/ublox/           UBX frame builder for receiver configuration
lib/imu/             unit conversion, zeroing, mounting compensation
lib/rc_protocol/     the RaceChrono BLE wire format
lib/kartcore/        session clock, button state machine, publishing pipeline
test/                host tests for everything under lib/
```

Nothing in `lib/` includes `Arduino.h`. The BLE transport sits behind an
interface, so the integration tests run the real pipeline against a recording
sink. See [docs/architecture.md](docs/architecture.md) for the reasoning.

## References

Both are included as git submodules under `reference/`:

- [renatobo/bonogps](https://github.com/renatobo/bonogps) — u-blox
  configuration sequences and NMEA rate handling.
- [aollin/racechrono-ble-diy-device](https://github.com/aollin/racechrono-ble-diy-device)
  — the RaceChrono DIY BLE API specification and a reference implementation.

Clone with them:

```bash
git clone --recurse-submodules <repository-url>
```

## License

MIT. See [LICENSE](LICENSE).

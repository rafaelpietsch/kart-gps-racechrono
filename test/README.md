# Tests

```bash
pio test -e native
```

Everything here runs on the host. No board is needed, and nothing in `lib/`
includes `Arduino.h`, which is what makes that possible — see
[docs/architecture.md](../docs/architecture.md).

| Suite | Covers |
| --- | --- |
| `test_rc_protocol` | The RaceChrono DIY BLE wire format: field encodings, the GPS packet layout, the sync counter, the CAN filter protocol |
| `test_nmea` | Sentence framing and checksums, field parsers, epoch assembly |
| `test_ublox` | UBX framing, checksums and the configuration messages |
| `test_imu` | Scale factors, burst decoding, rotation maths, zeroing, mounting compensation |
| `test_kartcore` | Button debouncing and press classification, rate limiting, the session clock, channel payloads |
| `test_integration` | The whole chain: recorded NMEA and synthetic register reads through the real pipeline to recorded BLE notifications |

## What the tests are checked against

Two suites deliberately compare against something other than this codebase, so
that a wrong-but-consistent implementation cannot pass:

- **`test_gps_main_packet_golden_vector`** builds the expected 20 bytes by hand
  from the specification in
  `reference/racechrono-ble-diy-device/README.md`, digit by digit. If the field
  order, the endianness or a scale factor is wrong, the test fails rather than
  agreeing with the bug.
- **`test_ublox`** compares every generated frame with the byte sequences
  published by the [bonogps](https://github.com/renatobo/bonogps) project. Those
  were produced by an independent implementation, so matching them is real
  evidence rather than a tautology.

## What the integration suite actually exercises

`test_integration` instantiates the real `SentenceAssembler`, `FixAssembler`,
`MotionProcessor` and `TelemetryPipeline`. Only the BLE transport is replaced,
by a `RecordingSink` that keeps every notification. Assertions are on decoded
wire bytes, not on internal state, so they stay honest if the internals are
refactored.

It covers the cases that only appear once the pieces are combined:

- recorded NMEA producing a correct GPS notification end to end,
- the GPS time characteristic staying quiet until the hour rolls over,
- nothing being published before RaceChrono writes its filter,
- the 25 Hz ceiling holding when the app asks for everything,
- a slower interval requested by the app being honoured,
- the SET button zeroing the session clock mid-run,
- a reconnect starting from "deny all" rather than inheriting the old
  subscription,
- one second of traffic fitting the BLE link budget.

## Notes for adding tests

Test names are sentences describing the behaviour, not the function under test:
`test_altitude_switches_to_coarse_above_the_fine_ceiling` rather than
`test_encodeAltitude_2`. When a test encodes a rule that came from a datasheet
or a specification, put the reasoning in a comment — the next person needs to
know whether an assertion is a real constraint or an arbitrary choice.

Time is always passed in explicitly. Nothing under test reads a clock, so there
are no sleeps and no flaky timing.

# Architecture

## The shape of it

```
   UART                     I2C
     |                       |
     v                       v
 GpsReceiver             Mpu6050            <-- src/hal, Arduino dependent
     |                       |
     v                       v
 SentenceAssembler      decodeBurst
 FixAssembler           toPhysicalUnits
     |                  BiasCalibrator
     |                  MotionProcessor
     |                       |
     +----------+------------+
                v
        TelemetryPipeline                   <-- lib/, no Arduino anywhere
         GpsEncoder, CanFilter
         SessionTimer, RateLimiter
                |
                v
           PacketSink  (interface)
                |
     +----------+-----------+
     v                      v
 BleTelemetryServer     RecordingSink       <-- device / tests
   (NimBLE)              (integration tests)
```

## The one decision everything else follows from

**Nothing under `lib/` includes `Arduino.h`.**

Embedded code that reaches for `millis()`, `Serial` or `Wire` in the middle of a
parser can only be tested on the device, and testing on the device means
flashing, driving and reading a log after the fact. That is a slow way to find
out that an altitude of 3000 m encodes as a negative number.

So every part that makes a decision — parsing a coordinate, choosing the fine or
coarse encoding, deciding whether a packet is due, classifying a button press —
takes its inputs as plain values, including the current time, and returns plain
values. The hardware layer in `src/hal/` reads registers and bytes and passes
them in. It contains no arithmetic worth testing.

The consequence is that `pio test -e native` compiles and exercises the entire
signal path on a desktop in a couple of seconds, and the integration suite can
assert on the exact bytes that would have gone over the air.

## Layers

### `lib/telemetry` — domain types

`GnssFix`, `ImuRawSample`, `ImuSample`. Every optional group carries an explicit
validity flag rather than a sentinel value: a 2D fix has a position but no
altitude, and the encoder needs to tell "zero metres" from "unknown".

### `lib/nmea` — bytes to a fix

Split in two. `SentenceAssembler` is a byte-at-a-time state machine that emits
checksum-validated sentence bodies and counts what it rejected. `FixAssembler`
merges the sentences of one navigation epoch into a `GnssFix`.

Coordinates are parsed with 64-bit integer arithmetic into degrees times 1e7.
Going through a 32-bit float would cost about a metre of resolution — visible as
a wobble in the racing line, which is the whole point of the device.

An epoch is complete when both RMC and GGA have arrived with the same
timestamp. u-blox emits GGA, then GSA, then RMC, so the fix is published on the
RMC with everything already merged, with no artificial one-epoch delay.

### `lib/ublox` — receiver configuration

Frames are built, not hardcoded, so a payload edit cannot leave a stale
checksum behind. The tests pin the output against the byte sequences published
by the bonogps project, which makes them a cross-check against an independent
implementation rather than a restatement of this code.

### `lib/imu` — physical units and mounting

Scale conversion, the sample rate divider, and the two pieces that make the SET
button meaningful:

- `BiasCalibrator` averages a window of stationary samples and **rejects** it if
  the readings moved, or if the total acceleration is not close to 1 g. A zero
  captured on a rolling kart is worse than no zero: it silently biases every lap
  that follows, and nothing downstream can detect it.
- `MotionProcessor` builds a rotation that maps the captured gravity vector onto
  +Z, so the box does not have to be mounted flat, and subtracts the gyro bias.
  Yaw is the one component gravity cannot supply, so it stays a build-time
  setting.

### `lib/rc_protocol` — the wire format

The RaceChrono DIY API is small but has sharp edges: the packet ID is
little-endian while everything else is big-endian, altitude and speed each have
a fine and a coarse encoding with a switchover that is easy to get wrong, and a
3-bit sync counter ties the two GPS characteristics together. Each of those has
its own test.

The fine encodings are masked to 15 bits, so altitude tops out at 2776.7 m and
speed at 327.67 km/h — not at the 6553.5 and 655.35 the full 16 bits would
suggest. Switching to the coarse form any later leaves a band of values that
encode and decode as a plausible but wrong number.

### `lib/kartcore` — scheduling and publishing

`SessionTimer`, `ButtonDebouncer`, `RateLimiter` and `TelemetryPipeline`. All
millisecond arithmetic is unsigned subtraction, which stays correct across the
32-bit rollover at 49.7 days of uptime. That is far longer than any session,
but it costs nothing to get right and there is a test for it.

`TelemetryPipeline` is where the rules live: publish nothing while nobody is
connected, respect both RaceChrono's per-PID interval and our own rate ceiling,
notify the GPS time characteristic only when the hour actually changes.

## Concurrency

There are exactly two threads: the Arduino loop and NimBLE's own task.

They meet at one place. `BleTelemetryServer` queues connection events and filter
writes from the NimBLE callbacks into a small ring buffer behind a spinlock, and
`drainEvents()` applies them from the loop. The pipeline itself is therefore
single threaded and needs no locks on the hot path.

The alternative — calling into the pipeline directly from the BLE callback —
would put a filter write in a race with a notification being encoded, for the
sake of a few hundred microseconds of latency on an event that happens a handful
of times per connection.

## Loop budget

At the default rates the loop has to service:

| Work | Rate | Cost |
| --- | --- | --- |
| GPS UART drain | ~1000 B/s | bounded at 512 bytes per call |
| MPU-6050 burst read | 100 Hz | 14 bytes over I2C at 400 kHz |
| Motion notification | 25 Hz | 16 byte packet |
| GPS notification | 5 Hz | 20 byte packet |
| Status notification | 1 Hz | 14 byte packet |

Roughly 31 notifications and under 900 bytes per second. A 15 ms connection
interval carries several notifications per event, so the design sits well inside
the link budget with room for retries. `test_one_second_of_telemetry_fits_the_ble_budget`
asserts this rather than leaving it as a claim.

## What is deliberately not here

- **No SD logging.** RaceChrono is the logger; duplicating it would mean
  reconciling two clocks.
- **No WiFi or web configuration.** bonogps does that well. Every knob here is a
  build flag, which keeps the radio, the RAM and the attack surface for a device
  that lives on a kart.
- **No sensor fusion into position.** Blending inertial data into the position
  track is RaceChrono's job, and it has far more context to do it with.
- **No interrupt-driven IMU sampling.** The INT line is not wired. Polling at
  100 Hz against a 25 Hz output has jitter well under one sample period, and it
  keeps the loop trivially reasonable about.

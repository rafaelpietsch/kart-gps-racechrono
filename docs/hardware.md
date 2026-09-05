# Hardware

## Bill of materials

| Part | Notes |
| --- | --- |
| ESP32-C3 Super Mini | ESP32-C3FH4, 4 MB flash, native USB, on-board LED on GPIO8 |
| GY-NEO6MV2 | u-blox NEO-6M with a ceramic patch antenna and a backup battery |
| GY-521 (MPU-6050) | 6-axis accelerometer and gyroscope |
| Momentary push button | SET, wired to ground |
| 5 V power source | USB power bank, or the kart's own supply through a regulator |

## Wiring

```
                ESP32-C3 Super Mini
                +------------------+
   GY-NEO6MV2   |                  |        GY-521 (MPU-6050)
   +--------+   |                  |        +---------+
   |    VCC |---| 3V3          3V3 |--------| VCC     |
   |    GND |---| GND          GND |--------| GND     |
   |     TX |---| GPIO20      GPIO5|--------| SDA     |
   |     RX |---| GPIO21      GPIO6|--------| SCL     |
   +--------+   |                  |        | AD0 --> GND (address 0x68)
                |                  |        +---------+
                | GPIO3 ---[SET]---| GND
                | GPIO8 --> on-board LED
                +------------------+
```

Both breakouts carry their own regulator and are happy on the 3V3 rail. Running
them from 5 V instead would put 5 V logic on the C3's inputs on some GY-521
clones, so keep both on 3V3.

### Why these pins

- **GPIO20 / GPIO21** are the pins silkscreened `RX` / `TX` on the Super Mini.
  The firmware drives the GPS from **UART1**, not UART0, and routes it to these
  pins through the GPIO matrix. UART0 also defaults here, so the ROM bootloader
  prints a few hundred bytes at 115200 on GPIO21 at every reset. The GPS ignores
  them, and the console lives on native USB instead, so nothing is lost.
- **GPIO5 / GPIO6** for I2C. The Arduino core's default C3 pins are GPIO8 and
  GPIO9, which are the LED and the BOOT strapping pin respectively; pulling
  either around with I2C pull-ups is asking for a board that will not boot.
- **GPIO3** for the button, with the internal pull-up. It is not a strapping pin
  and it is easy to reach on the header.
- **GPIO8** is the on-board LED, active low.

### Current draw

Roughly 150 mA average: about 45 mA for the NEO-6M while it acquires (35 mA
tracking), 4 mA for the MPU-6050 and 80 to 120 mA for the C3 with BLE active.
A 5000 mAh power bank covers a full track day with room to spare.

## GNSS configuration

The firmware reconfigures the receiver at **every boot**, on purpose. Most
GY-NEO6MV2 boards have no configuration EEPROM: the backup battery preserves the
ephemeris for a hot start, but the port and message settings revert to the
factory defaults — 9600 baud, 1 Hz, GGA + GLL + GSA + GSV + RMC + VTG — as soon
as the module loses power. Assuming otherwise produces a device that works on
the bench and silently falls back to 1 Hz at the track.

The sequence, in order:

1. `CFG-PRT` to move UART1 to 115200 baud.
2. Reopen the ESP32 UART at the new rate and repeat `CFG-PRT`, so a receiver
   that was already reconfigured ends up in the same known state.
3. `CFG-MSG` to enable GGA, RMC and GSA, and to silence GLL, VTG, GSV and GBS.
4. `CFG-NAV5` to select the automotive dynamic model.
5. `CFG-RATE` for the navigation rate.

### Why 9600 baud cannot carry 5 Hz

One epoch of the messages we keep is about 200 bytes:

| Sentence | Typical length |
| --- | --- |
| `GNRMC` | 70 B |
| `GNGGA` | 79 B |
| `GNGSA` | 50 B |

At 5 Hz that is 1000 B/s. UART 8N1 spends 10 bits on every byte, so the line
needs 10 000 bit/s of payload — already above 9600 baud before any framing
margin. The receiver's output would be truncated mid-sentence, which the parser
would correctly reject, and the fix rate would collapse.

38400 baud would be enough. 115200 is used instead to leave headroom for a
faster receiver and for the occasional burst, at no cost.

### Navigation rate ceiling

The NEO-6M accepts `CFG-RATE` down to a 200 ms period. Below that the u-blox 6
navigation engine cannot produce independent solutions, so the receiver either
NAKs the message or repeats the previous fix. The firmware defaults to 5 Hz and
exposes `KARTGPS_GPS_RATE_HZ` for receivers that can do better.

## Inertial sensor configuration

| Setting | Value | Why |
| --- | --- | --- |
| Accelerometer range | +/- 4 g | A kart pulls about 2 g lateral and 2.5 g braking. +/- 4 g keeps the full-scale resolution useful for the g-g diagram while leaving margin; the encoder saturates rather than wraps if a kerb exceeds it. |
| Gyroscope range | +/- 500 deg/s | Yaw rates in a kart spin stay well inside this, and it is four times the resolution of the +/- 2000 setting. |
| Low pass filter | 44 Hz | Cuts the engine and chassis vibration that would otherwise alias into the 25 Hz band. |
| Sample rate | 100 Hz | Four samples per published packet, so the filter has something to work with. |

Change any of them with a build flag, for example `-DKARTGPS_IMU_SAMPLE_HZ=200`.

## The SET button

| Action | Effect |
| --- | --- |
| Short press | Zero the accelerometer and gyroscope, restart the session clock |
| Hold 3 s | Everything above, plus an MPU-6050 signal path reset and a GNSS hot restart |

Zeroing averages 256 samples, about 2.5 seconds at 100 Hz. **The kart has to be
standing still and reasonably level.** The calibrator rejects the window if the
readings move more than 0.20 g or 10 deg/s peak to peak, or if the total
acceleration is not close to 1 g — a zero captured on a rolling kart biases
every lap that follows, so refusing is better than accepting.

The capture also records the direction of gravity, which is used to level the
sensor. The box does not have to be mounted flat: whatever attitude it is in
when SET is pressed becomes "level". Yaw cannot be recovered from gravity, so if
the box does not face forwards, set `KARTGPS_MOUNTING_YAW_DEGREES`.

A calibration is held in RAM and is lost on power-down, which is why the
firmware takes one automatically at boot.

## Status LED

| Pattern | Meaning |
| --- | --- |
| Solid | Connected to RaceChrono with a live fix |
| 1 Hz blink, even | Connected, but no usable GNSS fix |
| Short wink once a second | Advertising, nothing connected |
| 10 Hz flicker | Zeroing in progress, hold still |
| Two quick flashes, pause | Zeroing was rejected, or the MPU-6050 did not answer |

The two-flash pattern covers two faults that need opposite fixes, so do not
guess between them: open the serial console and press `s`, which names the one
that is actually firing.

## Serial console

The board speaks over its native USB CDC, and that port throws away everything
written before a host opens it. `pio device monitor` almost always attaches
after `setup()` has run, so the boot log — the one place that says whether the
MPU-6050 answered — is routinely lost. Resetting the board with the monitor
already open brings it back, and so does the console.

Open the monitor and press a key:

| Key | What it does |
| --- | --- |
| `s` | Status: what the LED means right now, and every counter behind it |
| `b` | Replays the boot report, including the raw WHO_AM_I read |
| `i` | I2C: idle line levels, an address scan, and an MPU-6050 register dump |
| `n` | Toggles a raw echo of the GPS UART |
| `z` | Re-runs the zeroing |
| `r` | Re-probes the MPU-6050 without a reboot |
| `?` | Help |

### Reading an I2C scan

`i` separates the three faults that all look like "the sensor is missing":

- **A line reads LOW when idle.** No pull-up, a short to ground, or a device
  holding the bus. Nothing will be found until that clears. The GY-521 carries
  its own pull-ups, so this usually means SDA and SCL are swapped with
  something else, or the module has no 3V3.
- **Every address NACKs cleanly, none found.** The bus works electrically but
  nothing is answering: check power to the module first, then the SDA/SCL pair.
- **A device answers, but at 0x69.** AD0 is tied high on that breakout. Build
  with `-DKARTGPS_MPU_ADDRESS=0x69`, or ground AD0.

### Reading a raw UART echo

`n` mirrors every byte the GPS sends, printable characters as themselves and
everything else as `<XX>`. That distinction is the whole point:

- **Nothing at all**, and `rxBytes` stays at zero: the receiver is not talking.
  A wiring or power fault, not a parsing one.
- **Legible `$GPRMC,...` lines**: the link is fine, and any missing fix is the
  receiver still searching for satellites.
- **Mostly `<XX>` hex**: bytes arrive at the wrong baud rate. The receiver
  ignored the `CFG-PRT` that moves it to 115200, which a clone module with a
  non-default port configuration will do.

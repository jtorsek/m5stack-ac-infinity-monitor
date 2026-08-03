# M5Stack Core -- Multi-Device AC Infinity CloudCom Monitor

Standalone PlatformIO firmware for the M5Stack Core (classic ESP32,
ILI9341 320x240 screen, three physical buttons). Once flashed, the board
passively scans for several AC Infinity CloudCom A1 devices at once and
cycles the display through each one every 5 seconds, showing temperature,
humidity, VPD (vapor pressure deficit), battery level, and the device's
own name -- no phone, computer, or BLE dongle needed. Press the leftmost
button to see a graph of the last 24 hours of temperature history for
whichever device is on screen.

Ported from [lilygo-ac-infinity-monitor](https://github.com/jtorsek/lilygo-ac-infinity-monitor)
(same feature set on a LilyGo T-Display S3) -- same decode logic, same
`devices.h` format, adapted here for the M5Stack Core's screen/button
hardware and extended with the history graph. Both share the decode logic
originally ported from the Python scanner in
[ac-infinity-bleuio](https://github.com/jtorsek/ac-infinity-bleuio) (full
write-up on how the temperature/humidity/VPD formulas were field-verified
lives there).

## Configuring your devices

Devices are listed in `include/devices.h`:

```cpp
static const KnownDevice KNOWN_DEVICES[] = {
    {"a4:c1:38:8e:05:33", 0, "Not Used"},
    {"a4:c1:38:82:a3:9a", 0, "Sniffer -Bleu.io"},
    {"a4:c1:38:0f:7f:15", 0, "Kontoret"},
    {"a4:c1:38:33:f5:be", 0, "Kylskapet"},
    {"a4:c1:38:fc:9d:04", 0, "Svamplådan"},
};
```

Names can use å/ä/ö/Å/Ä/Ö (or any other UTF-8 text) -- see "Character
rendering" below for how that works.

To add a new device, find its MAC address with
`ac_infinity_bleuio_scanner.py --watch` from the ac-infinity-bleuio repo,
add a line here, and reflash -- no other code changes needed.

## Controls

- **Button A** (leftmost): toggle the history graph for whichever device
  is currently on screen. Live cycling pauses while the graph is open.
- **Button B / C**: step to the previous/next device -- works in both live
  view (picking one manually pauses the 5-second auto-cycling until you
  let it run again) and in graph mode (switches whose history is plotted).
- **Button A** again: back to live cycling.

## History graph

Each device gets its own ring buffer of one temperature sample per minute,
holding the last 1440 samples (24 hours). It lives in RAM (under 6KB per
device -- negligible next to the M5Stack Core's 512KB), but is also
mirrored to the SD card after every new sample and reloaded at boot, so a
reboot or power cycle doesn't lose it. Each device gets its own file,
named after its MAC address (e.g. `/hist_a4c1388e0533.bin`), so history
stays matched to the right device even if `devices.h` is reordered later.

If no SD card is inserted, the firmware detects that at boot, logs it,
and simply runs with RAM-only history for that session -- no SD card is
not treated as an error. If `HISTORY_SIZE` ever changes in a future
firmware update, old files from a different size are detected (via a
header written alongside the data) and ignored rather than misread.

M5's own `M5.begin()` already tries `SD.begin()` once at a fixed 40MHz,
which some cards/wiring can't sustain reliably -- on real hardware this
showed up as `f_mount failed: (1) A hard error occurred in the low level
disk I/O layer` even with a correctly FAT32-formatted card. The firmware
retries once at the SD library's conservative default speed (4MHz)
before giving up and falling back to RAM-only.

The plotted line is downsampled to one point per pixel column, so it
stays fast and legible regardless of how many samples are buffered.

## Display cycling (live view)

- Each configured device gets 5 seconds on screen before the display moves
  to the next one, showing the device's name, its position in the
  rotation (e.g. `(2/4)`), and its latest reading.
- A device that hasn't been heard from yet shows "Waiting for signal...".
- A device that was seen before but hasn't advertised in the last 10
  seconds shows "Lost signal, retrying..." -- this is purely a per-device
  display state; the scan itself keeps running continuously for all
  devices in the background.

## Character rendering

All text is drawn via [U8g2_for_Adafruit_GFX](https://github.com/olikraus/U8g2_for_Adafruit_GFX)
instead of M5Stack's/TFT_eSPI's own fonts -- those built-in fonts are
ASCII-only, so å/ä/ö (and any other non-ASCII text) would come out as
missing or garbled characters. `TFTGFXAdapter` in `src/main.cpp` is the
minimal bridge that lets U8g2 draw onto `M5.Lcd` (an `M5Display`, which
is itself a `TFT_eSPI`) rather than switching the whole UI to
Adafruit_GFX. Same fix as in the (unrelated) zaptec-cyd-charger project.

## WiFi + OTA updates

Connects to WiFi at boot (best-effort, with a 10s timeout) so firmware
can be updated wirelessly afterwards -- BLE scanning, the SD card, and
the display all work identically with no WiFi at all, and a
failed/missing connection is logged, not treated as an error. To enable
it:

1. Copy `include/config.h.example` to `include/config.h` and fill in your
   WiFi SSID/password and an OTA password (`config.h` is gitignored --
   never commit real credentials).
2. Flash once over USB as usual so the new firmware (with WiFi/OTA
   support) is running.
3. From then on, update over WiFi instead of USB:
   ```bash
   OTA_PASSWORD=<same value as in config.h> pio run -e m5stack_ota -t upload
   ```
   Reachable at `ac-infinity-m5stack.local` (or whatever `OTA_HOSTNAME` is
   set to) once it's on the network.

This board's own default partition table is sized for 4MB flash (each
OTA app slot only ~1.3MB) even though it has 16MB -- `platformio.ini`
overrides it with `default_16MB.csv` so each OTA slot gets ~6.25MB
instead, comfortably fitting the firmware (~1.2MB) with room to grow.
Changing the partition table needs a full reflash over USB (not OTA) to
take effect, since it's already baked into this repo's `platformio.ini`
that's a one-time thing, not something you'll hit again.

## Build and flash

```bash
pio run --target upload
```

## Requirements

- [PlatformIO](https://platformio.org/)
- An M5Stack Core (Basic/Gray/Fire/Core ESP32-16M -- any variant with the
  classic 3-button, ILI9341 320x240 form factor)
- One or more AC Infinity CloudCom A1 devices (or others using the same
  "C_1B" advertisement layout) within BLE range
- A microSD card (FAT32), optional -- only needed for history to survive
  a reboot; the firmware runs fine without one, just without persistence

## Protocol notes

Temperature, humidity, and VPD formulas are ported from
`decode_c1_mini_device()` and `compute_vpd_kpa()` in
`ac_infinity_bleuio_scanner.py` (ac-infinity-bleuio repo):

- Temperature: `temp_C = 1.6 * byte15 + 0.6`, accurate to ~1 degree C.
- Humidity: `hum_pct = 0.1065 * byte17 + 24.46`, accurate to ~0.7% RH.
- VPD: the same Tetens-equation formula AC Infinity's own app uses, with a
  leaf-temperature offset of 0.

See the ac-infinity-bleuio repo's `ac_infinity_bleuio_scanner.py`
docstring for the full field data and known caveats.

## Battery level

Unlike temperature/humidity/VPD, battery level isn't part of the passive
advertisement -- each CloudCom A1 exposes it as a standard Bluetooth SIG
Battery Service (`0x180F`) / Battery Level characteristic (`0x2A19`).
Reading it needs an actual GATT connection, so the firmware briefly stops
its passive scan, connects to one device, reads the value, disconnects,
and resumes scanning -- one device at a time, round robin, every 5
minutes.

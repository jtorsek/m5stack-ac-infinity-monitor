#pragma once

// Known AC Infinity devices to cycle through on the display. Add or remove
// entries here as you buy more sensors -- no other code changes needed,
// just reflash after editing this file.
//
// Get a new device's MAC address with ac_infinity_bleuio_scanner.py from
// the ac-infinity-bleuio repo (run it with --watch and look for the new
// device's "ABCDEFG"-named entry; match it to the app by its type_name,
// e.g. "JFD31", which corresponds to the app's "C-JFD31" label).
//
// name is shown on screen so you know which sensor's reading you're
// looking at -- keep these reasonably short, the screen is only 170px
// wide in portrait mode.

struct KnownDevice {
    const char *mac;     // lowercase, e.g. "a4:c1:38:8e:05:33"
    uint8_t addrType;    // 0 = public, 1 = random -- every CloudCom A1 seen so far uses public (0)
    const char *name;
};

static const KnownDevice KNOWN_DEVICES[] = {
    {"a4:c1:38:8e:05:33", 0, "Not Used"},
    {"a4:c1:38:82:a3:9a", 0, "Sniffer -Bleu.io"},
    {"a4:c1:38:0f:7f:15", 0, "Kontoret"},
    {"a4:c1:38:33:f5:be", 0, "Kylskapet"},
    {"a4:c1:38:fc:9d:04", 0, "Svamp ladan"},
};

static const size_t KNOWN_DEVICE_COUNT = sizeof(KNOWN_DEVICES) / sizeof(KNOWN_DEVICES[0]);

/*
 * M5Stack Core -- multi-device AC Infinity CloudCom monitor with a
 * button-triggered historical temperature graph.
 *
 * Same passive multi-device BLE scan/decode as lilygo-ac-infinity-monitor
 * (one continuous scan covers every configured device; each result is
 * matched against devices.h by MAC and decoded). Adapted here for the
 * M5Stack Core's ILI9341 screen (320x240 landscape) and its three
 * physical buttons:
 *
 *   - Button A (leftmost): toggle the history graph for whichever device
 *     is currently on screen. Live cycling pauses while the graph is up.
 *   - Button B / C: step to the previous/next device -- in both live view
 *     (pauses auto-cycling once you've picked one manually) and in graph
 *     mode (switches whose history is plotted).
 *
 * History is a 1-sample-per-minute ring buffer per device, holding the
 * last 24 hours (1440 points). It lives in RAM but is mirrored to the SD
 * card (one file per device, keyed by MAC address) after every new
 * sample, and reloaded from there at boot -- so a reboot or power cycle
 * doesn't lose it. If no SD card is present, the firmware just runs with
 * RAM-only history for that session (logged once at boot, not treated as
 * an error).
 *
 * Ported from https://github.com/jtorsek/lilygo-ac-infinity-monitor --
 * same decode formulas, same devices.h format. See that repo's README for
 * the field-verification background on the temp/humidity/VPD math.
 */

#include <M5Stack.h>
#include <NimBLEDevice.h>
#include <SD.h>
#include <math.h>
#include "devices.h"

static_assert(KNOWN_DEVICE_COUNT > 0, "Add at least one device to include/devices.h");

static const uint16_t AC_INFINITY_MANUFACTURER_ID = 0x0902;

// ---------------------------------------------------------------------
// AC Infinity CloudCom A1 (device type C_1B) advertisement decode --
// identical formulas to lilygo-ac-infinity-monitor / ac_infinity_bleuio_scanner.py.
// ---------------------------------------------------------------------
struct AcInfinityReading {
    bool valid = false;
    uint32_t lastUpdateMs = 0;
    int rssi = 0;
    float temperatureC = 0;
    float humidityPct = 0;
    float vpdKpa = 0;
};

static AcInfinityReading g_readings[KNOWN_DEVICE_COUNT];

static const char *BATTERY_SERVICE_UUID = "180F";
static const char *BATTERY_LEVEL_CHAR_UUID = "2A19";
static bool g_batteryValid[KNOWN_DEVICE_COUNT];
static uint8_t g_batteryPct[KNOWN_DEVICE_COUNT];

static float saturationVaporPressureKpa(float tempC) {
    return 0.61078f * expf(17.2694f * tempC / (tempC + 238.3f));
}

static bool decodeAcInfinityPayload(const uint8_t *payload, size_t len, AcInfinityReading &out) {
    if (len < 18) {
        return false;
    }
    uint8_t tempRaw = payload[15];
    uint8_t humRaw = payload[17];
    float tempC = 1.6f * tempRaw + 0.6f;
    float humPct = 0.1065f * humRaw + 24.46f;
    if (tempC < -20.0f || tempC > 60.0f || humPct < 0.0f || humPct > 100.0f) {
        return false;
    }
    out.temperatureC = tempC;
    out.humidityPct = humPct;
    float svp = saturationVaporPressureKpa(tempC);
    float vpd = svp - svp * humPct / 100.0f;
    out.vpdKpa = vpd < 0 ? 0 : vpd;
    return true;
}

static int findDeviceIndex(const NimBLEAddress &addr) {
    for (size_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
        NimBLEAddress known(KNOWN_DEVICES[i].mac,
                             KNOWN_DEVICES[i].addrType == 0 ? BLE_ADDR_PUBLIC : BLE_ADDR_RANDOM);
        if (addr.equals(known)) {
            return (int)i;
        }
    }
    return -1;
}

// ---------------------------------------------------------------------
// BLE scanning -- ONE continuous scan covers every configured device at
// once (unlike the GATT battery check below, which must target one
// address at a time).
// ---------------------------------------------------------------------
static NimBLEScan *g_scan;

class AcInfinityScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *device) override {
        int idx = findDeviceIndex(device->getAddress());
        if (idx < 0) {
            return;
        }
        if (!device->haveManufacturerData()) {
            return;
        }

        uint8_t count = device->getManufacturerDataCount();
        for (uint8_t i = 0; i < count; i++) {
            std::string raw = device->getManufacturerData(i);
            if (raw.size() < 2) {
                continue;
            }
            const uint8_t *bytes = reinterpret_cast<const uint8_t *>(raw.data());
            uint16_t companyId = bytes[0] | (bytes[1] << 8);
            if (companyId != AC_INFINITY_MANUFACTURER_ID) {
                continue;
            }

            const uint8_t *payload = bytes + 2;
            size_t payloadLen = raw.size() - 2;

            AcInfinityReading reading;
            if (!decodeAcInfinityPayload(payload, payloadLen, reading)) {
                continue;
            }
            reading.valid = true;
            reading.lastUpdateMs = millis();
            reading.rssi = device->getRSSI();
            g_readings[idx] = reading;
            Serial.printf("[%s] %.1f C, %.1f %%, %.2f kPa, RSSI %d\n",
                          KNOWN_DEVICES[idx].name, reading.temperatureC, reading.humidityPct,
                          reading.vpdKpa, reading.rssi);
            return;
        }
    }
} g_scanCallbacks;

// ---------------------------------------------------------------------
// History -- last 24 hours of temperature, one sample per minute, per device.
// ---------------------------------------------------------------------
static const size_t HISTORY_SIZE = 24 * 60;
static const uint32_t HISTORY_INTERVAL_MS = 60UL * 1000UL;

struct History {
    float samples[HISTORY_SIZE];
    uint16_t count = 0; // valid samples so far, caps at HISTORY_SIZE
    uint16_t head = 0;  // index the next sample will be written to
};
static History g_history[KNOWN_DEVICE_COUNT];
static uint32_t g_lastHistoryRecordMs = 0;

// ---------------------------------------------------------------------
// SD card persistence -- one file per device (named after its MAC, since
// devices.h entries can be reordered/added), holding a small header plus
// the raw History contents. Written after every new sample and reloaded
// at boot, so history survives a reboot/power cycle.
// ---------------------------------------------------------------------
static bool g_sdReady = false;
static const uint32_t HISTORY_FILE_MAGIC = 0x48434941; // "AICH" -- AC Infinity/M5 history
static const uint16_t HISTORY_FILE_VERSION = 1;

static String historyFilePath(size_t idx) {
    String path = "/hist_";
    for (const char *p = KNOWN_DEVICES[idx].mac; *p; p++) {
        if (*p != ':') {
            path += *p;
        }
    }
    path += ".bin";
    return path;
}

static void saveHistoryToSd(size_t idx) {
    if (!g_sdReady) {
        return;
    }
    String path = historyFilePath(idx);
    SD.remove(path); // FILE_WRITE appends rather than truncates -- remove first for a clean overwrite
    File f = SD.open(path, FILE_WRITE);
    if (!f) {
        Serial.printf("Failed to open %s for writing\n", path.c_str());
        return;
    }
    const History &h = g_history[idx];
    uint16_t size = HISTORY_SIZE;
    f.write((const uint8_t *)&HISTORY_FILE_MAGIC, sizeof(HISTORY_FILE_MAGIC));
    f.write((const uint8_t *)&HISTORY_FILE_VERSION, sizeof(HISTORY_FILE_VERSION));
    f.write((const uint8_t *)&size, sizeof(size));
    f.write((const uint8_t *)&h.count, sizeof(h.count));
    f.write((const uint8_t *)&h.head, sizeof(h.head));
    f.write((const uint8_t *)h.samples, sizeof(h.samples));
    f.close();
}

static void loadHistoryFromSd(size_t idx) {
    if (!g_sdReady) {
        return;
    }
    String path = historyFilePath(idx);
    if (!SD.exists(path)) {
        return;
    }
    File f = SD.open(path, FILE_READ);
    if (!f) {
        Serial.printf("Failed to open %s for reading\n", path.c_str());
        return;
    }

    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t size = 0;
    History &h = g_history[idx];
    bool ok = f.read((uint8_t *)&magic, sizeof(magic)) == sizeof(magic) &&
              f.read((uint8_t *)&version, sizeof(version)) == sizeof(version) &&
              f.read((uint8_t *)&size, sizeof(size)) == sizeof(size) &&
              magic == HISTORY_FILE_MAGIC && version == HISTORY_FILE_VERSION && size == HISTORY_SIZE;
    if (ok) {
        ok = f.read((uint8_t *)&h.count, sizeof(h.count)) == sizeof(h.count) &&
             f.read((uint8_t *)&h.head, sizeof(h.head)) == sizeof(h.head) &&
             f.read((uint8_t *)h.samples, sizeof(h.samples)) == sizeof(h.samples);
    }
    f.close();

    if (ok) {
        Serial.printf("Loaded %u history sample(s) for %s from SD\n", (unsigned)h.count,
                      KNOWN_DEVICES[idx].name);
    } else {
        // Missing/old-format/wrong-size file (e.g. HISTORY_SIZE changed since
        // it was written) -- ignore it and start fresh rather than trusting
        // whatever partial data was read.
        Serial.printf("Ignoring incompatible history file for %s\n", KNOWN_DEVICES[idx].name);
        h.count = 0;
        h.head = 0;
    }
}

static void recordHistory() {
    for (size_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
        if (!g_readings[i].valid) {
            continue;
        }
        History &h = g_history[i];
        h.samples[h.head] = g_readings[i].temperatureC;
        h.head = (h.head + 1) % HISTORY_SIZE;
        if (h.count < HISTORY_SIZE) {
            h.count++;
        }
        saveHistoryToSd(i);
    }
}

// ---------------------------------------------------------------------
// Display -- live cycling mode + history graph mode (button A toggles).
// ---------------------------------------------------------------------
static const uint32_t DISPLAY_CYCLE_MS = 5000;
static size_t g_displayIndex = 0;
static uint32_t g_displaySwitchedAtMs = 0;
static const uint32_t LOST_SIGNAL_AFTER_MS = 10000;

static bool g_graphMode = false;
static size_t g_graphIndex = 0;

static void drawValue(int x, int y, const char *label, const char *value, uint16_t color) {
    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(x, y);
    M5.Lcd.print(label);

    M5.Lcd.setTextColor(color, BLACK);
    M5.Lcd.setTextFont(4);
    M5.Lcd.setCursor(x, y + 18);
    M5.Lcd.print(value);
}

static void renderLive() {
    const AcInfinityReading &reading = g_readings[g_displayIndex];
    const KnownDevice &device = KNOWN_DEVICES[g_displayIndex];

    M5.Lcd.fillScreen(BLACK);

    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(6, 6);
    M5.Lcd.print(device.name);

    char posBuf[16];
    snprintf(posBuf, sizeof(posBuf), "(%u/%u)", (unsigned)(g_displayIndex + 1), (unsigned)KNOWN_DEVICE_COUNT);
    int posWidth = M5.Lcd.textWidth(posBuf);
    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.setCursor(320 - 6 - posWidth, 6);
    M5.Lcd.print(posBuf);

    if (!reading.valid) {
        M5.Lcd.setTextFont(2);
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.setCursor(6, 40);
        M5.Lcd.print("Waiting for signal...");
    } else if (millis() - reading.lastUpdateMs > LOST_SIGNAL_AFTER_MS) {
        M5.Lcd.setTextFont(2);
        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.setCursor(6, 40);
        M5.Lcd.print("Lost signal, retrying...");
    } else {
        char buf[32];
        const int col1 = 6, col2 = 168;
        int y = 34;

        snprintf(buf, sizeof(buf), "%.1f C", reading.temperatureC);
        drawValue(col1, y, "Temperature", buf, WHITE);

        snprintf(buf, sizeof(buf), "%.1f %%", reading.humidityPct);
        drawValue(col2, y, "Humidity", buf, WHITE);

        y += 60;
        snprintf(buf, sizeof(buf), "%.2f kPa", reading.vpdKpa);
        drawValue(col1, y, "VPD", buf, WHITE);

        if (g_batteryValid[g_displayIndex]) {
            snprintf(buf, sizeof(buf), "%d %%", g_batteryPct[g_displayIndex]);
            drawValue(col2, y, "Battery", buf, g_batteryPct[g_displayIndex] <= 20 ? ORANGE : WHITE);
        }

        y += 60;
        uint32_t ageSec = (millis() - reading.lastUpdateMs) / 1000;
        snprintf(buf, sizeof(buf), "live, %lus ago, %d dBm", (unsigned long)ageSec, reading.rssi);
        M5.Lcd.setTextColor(DARKGREY, BLACK);
        M5.Lcd.setTextFont(2);
        M5.Lcd.setCursor(6, y);
        M5.Lcd.print(buf);
    }

    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.setTextFont(1);
    M5.Lcd.setCursor(6, 224);
    M5.Lcd.print("A: history graph   B/C: select sensor");
}

static void renderGraph() {
    const History &h = g_history[g_graphIndex];
    const KnownDevice &device = KNOWN_DEVICES[g_graphIndex];

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setTextFont(2);
    M5.Lcd.setCursor(6, 6);
    M5.Lcd.print(device.name);
    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.setCursor(6, 24);
    M5.Lcd.print("Temperature, last 24 hours");

    const int plotX = 40, plotY = 46, plotW = 270, plotH = 140;
    M5.Lcd.drawRect(plotX, plotY, plotW, plotH, DARKGREY);

    if (h.count < 2) {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.setTextFont(2);
        M5.Lcd.setCursor(plotX + 10, plotY + plotH / 2 - 8);
        M5.Lcd.print("Not enough history yet");
    } else {
        float minT = 1000, maxT = -1000;
        for (uint16_t i = 0; i < h.count; i++) {
            uint16_t idx = (h.head + HISTORY_SIZE - h.count + i) % HISTORY_SIZE;
            float v = h.samples[idx];
            if (v < minT) minT = v;
            if (v > maxT) maxT = v;
        }
        if (maxT - minT < 1.0f) {
            float mid = (maxT + minT) / 2;
            minT = mid - 0.5f;
            maxT = mid + 0.5f;
        }

        char buf[16];
        M5.Lcd.setTextFont(1);
        M5.Lcd.setTextColor(DARKGREY, BLACK);
        snprintf(buf, sizeof(buf), "%.1f", maxT);
        M5.Lcd.setCursor(2, plotY);
        M5.Lcd.print(buf);
        snprintf(buf, sizeof(buf), "%.1f", minT);
        M5.Lcd.setCursor(2, plotY + plotH - 8);
        M5.Lcd.print(buf);

        // Downsample to one point per pixel column so plotting stays fast
        // and legible regardless of how many samples are buffered (up to
        // HISTORY_SIZE for a full 24h at 1-minute resolution).
        int prevX = -1, prevY = -1;
        for (int col = 0; col < plotW; col++) {
            uint16_t sampleIdx = (uint32_t)col * (h.count - 1) / (plotW - 1);
            uint16_t idx = (h.head + HISTORY_SIZE - h.count + sampleIdx) % HISTORY_SIZE;
            float v = h.samples[idx];
            int x = plotX + col;
            int y = plotY + plotH - (int)((v - minT) / (maxT - minT) * plotH);
            if (prevX >= 0) {
                M5.Lcd.drawLine(prevX, prevY, x, y, GREEN);
            }
            prevX = x;
            prevY = y;
        }
    }

    M5.Lcd.setTextColor(DARKGREY, BLACK);
    M5.Lcd.setTextFont(1);
    M5.Lcd.setCursor(plotX, plotY + plotH + 4);
    M5.Lcd.print("-24h");
    const char *nowLabel = "now";
    int nowWidth = M5.Lcd.textWidth(nowLabel);
    M5.Lcd.setCursor(plotX + plotW - nowWidth, plotY + plotH + 4);
    M5.Lcd.print(nowLabel);

    M5.Lcd.setCursor(6, 224);
    M5.Lcd.print("A: back   B: prev device   C: next device");
}

// ---------------------------------------------------------------------
// Battery level -- read via a brief GATT connection (not in the passive
// advertisement), one configured device at a time in rotation so we don't
// interrupt live scanning for too long in one go.
// ---------------------------------------------------------------------
static uint32_t g_lastBatteryCheckMs = 0;
static size_t g_nextBatteryCheckIndex = 0;
static const uint32_t BATTERY_CHECK_INTERVAL_MS = 5UL * 60UL * 1000UL;

static void checkBatteryLevel(size_t idx) {
    Serial.printf("Checking battery level for %s...\n", KNOWN_DEVICES[idx].name);
    g_scan->stop();

    NimBLEClient *client = NimBLEDevice::createClient();
    NimBLEAddress addr(KNOWN_DEVICES[idx].mac,
                        KNOWN_DEVICES[idx].addrType == 0 ? BLE_ADDR_PUBLIC : BLE_ADDR_RANDOM);
    if (client->connect(addr)) {
        NimBLERemoteService *service = client->getService(NimBLEUUID(BATTERY_SERVICE_UUID));
        NimBLERemoteCharacteristic *ch =
            service ? service->getCharacteristic(NimBLEUUID(BATTERY_LEVEL_CHAR_UUID)) : nullptr;
        if (ch && ch->canRead()) {
            std::string val = ch->readValue();
            if (val.size() >= 1) {
                g_batteryPct[idx] = (uint8_t)val[0];
                g_batteryValid[idx] = true;
                Serial.printf("Battery [%s]: %d%%\n", KNOWN_DEVICES[idx].name, g_batteryPct[idx]);
            }
        } else {
            Serial.println("Battery service/characteristic not found");
        }
        client->disconnect();
    } else {
        Serial.printf("Battery check failed to connect to %s\n", KNOWN_DEVICES[idx].name);
    }
    NimBLEDevice::deleteClient(client);

    g_scan->start(0, false, true);
}

// ---------------------------------------------------------------------
void setup() {
    M5.begin(); // also initializes the SD card (SDEnable defaults to true)
    Serial.begin(115200);
    delay(300);
    Serial.println("AC Infinity multi-device monitor (M5Stack) booting...");
    Serial.printf("Watching %u configured device(s):\n", (unsigned)KNOWN_DEVICE_COUNT);
    for (size_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
        Serial.printf("  %s -- %s\n", KNOWN_DEVICES[i].mac, KNOWN_DEVICES[i].name);
    }

    g_sdReady = SD.cardType() != CARD_NONE;
    if (g_sdReady) {
        Serial.println("SD card present -- history will persist across reboots");
        for (size_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
            loadHistoryFromSd(i);
        }
    } else {
        Serial.println("No SD card found -- history will be RAM-only this session");
    }

    M5.Lcd.fillScreen(BLACK);

    NimBLEDevice::init("");
    g_scan = NimBLEDevice::getScan();
    g_scan->setScanCallbacks(&g_scanCallbacks, true); // wantDuplicates=true -- see lilygo-ac-infinity-monitor's README
    g_scan->setActiveScan(true); // required -- CloudCom's manufacturer data is in the scan response
    g_scan->setInterval(100);
    g_scan->setWindow(100);
    g_scan->start(0, false, true);
    Serial.println("Scanning...");

    g_displaySwitchedAtMs = millis();
}

void loop() {
    M5.update();

    bool needsRedraw = false;

    if (M5.BtnA.wasPressed()) {
        g_graphMode = !g_graphMode;
        needsRedraw = true;
        if (g_graphMode) {
            g_graphIndex = g_displayIndex;
            Serial.println("Entered history graph mode");
        } else {
            Serial.println("Back to live view");
            g_displaySwitchedAtMs = millis();
        }
    }

    if (M5.BtnB.wasPressed()) {
        if (g_graphMode) {
            g_graphIndex = (g_graphIndex + KNOWN_DEVICE_COUNT - 1) % KNOWN_DEVICE_COUNT;
        } else {
            g_displayIndex = (g_displayIndex + KNOWN_DEVICE_COUNT - 1) % KNOWN_DEVICE_COUNT;
            g_displaySwitchedAtMs = millis(); // manual pick -- restart the auto-cycle countdown
        }
        needsRedraw = true;
    }
    if (M5.BtnC.wasPressed()) {
        if (g_graphMode) {
            g_graphIndex = (g_graphIndex + 1) % KNOWN_DEVICE_COUNT;
        } else {
            g_displayIndex = (g_displayIndex + 1) % KNOWN_DEVICE_COUNT;
            g_displaySwitchedAtMs = millis();
        }
        needsRedraw = true;
    }

    if (!g_graphMode && millis() - g_displaySwitchedAtMs > DISPLAY_CYCLE_MS) {
        g_displayIndex = (g_displayIndex + 1) % KNOWN_DEVICE_COUNT;
        g_displaySwitchedAtMs = millis();
        needsRedraw = true;
    }

    if (millis() - g_lastHistoryRecordMs > HISTORY_INTERVAL_MS) {
        recordHistory();
        g_lastHistoryRecordMs = millis();
    }

    if (millis() - g_lastBatteryCheckMs > BATTERY_CHECK_INTERVAL_MS) {
        for (size_t attempt = 0; attempt < KNOWN_DEVICE_COUNT; attempt++) {
            size_t idx = (g_nextBatteryCheckIndex + attempt) % KNOWN_DEVICE_COUNT;
            if (g_readings[idx].valid) {
                checkBatteryLevel(idx);
                g_nextBatteryCheckIndex = (idx + 1) % KNOWN_DEVICE_COUNT;
                break;
            }
        }
        g_lastBatteryCheckMs = millis();
        needsRedraw = true;
    }

    static uint32_t lastRenderMs = 0;
    if (needsRedraw || millis() - lastRenderMs > 1000) {
        if (g_graphMode) {
            renderGraph();
        } else {
            renderLive();
        }
        lastRenderMs = millis();
    }

    delay(30);
}

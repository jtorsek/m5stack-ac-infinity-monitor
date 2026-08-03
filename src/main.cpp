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
 * Text is drawn via U8g2_for_Adafruit_GFX instead of M5Stack's own fonts
 * -- TFT_eSPI's (and so M5Stack's) built-in fonts are ASCII-only (no
 * å/ä/ö), which U8g2's fonts include, so device names in devices.h can
 * use proper Swedish spelling. Same fix as in the zaptec-cyd-charger
 * project; see TFTGFXAdapter below for the minimal bridge that makes
 * U8g2 draw onto M5.Lcd (an M5Display, which is itself a TFT_eSPI)
 * instead of switching the whole UI to Adafruit_GFX.
 *
 * Also connects to WiFi (best-effort, non-fatal if it fails or isn't
 * configured) to serve OTA firmware updates -- see include/config.h.example
 * and the m5stack_ota environment in platformio.ini. BLE scanning, the SD
 * card, and the display all work fine with no WiFi at all.
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
#include <Adafruit_GFX.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <math.h>
#include "devices.h"
#include "config.h"

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

// U8g2_for_Adafruit_GFX needs an Adafruit_GFX-derived object to call
// drawPixel() on, but this project draws with M5.Lcd (a TFT_eSPI
// subclass) directly everywhere else. This adapter is the minimal
// bridge: it forwards the one method U8g2_for_Adafruit_GFX actually
// calls to M5.Lcd, so we get U8g2's fonts (which include Swedish
// å/ä/ö/Å/Ä/Ö, unlike TFT_eSPI's built-in fonts) without switching the
// rest of the UI off M5.Lcd.
class TFTGFXAdapter : public Adafruit_GFX {
public:
    TFTGFXAdapter(TFT_eSPI &tft, int16_t w, int16_t h) : Adafruit_GFX(w, h), _tft(tft) {}
    void drawPixel(int16_t x, int16_t y, uint16_t color) override { _tft.drawPixel(x, y, color); }

private:
    TFT_eSPI &_tft;
};

static TFTGFXAdapter gfxAdapter(M5.Lcd, 320, 240);
static U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;

static const uint8_t *kFontTiny = u8g2_font_helvR08_tf;   // status bar, footer, axis labels
static const uint8_t *kFontSmall = u8g2_font_helvR12_tf;  // header, card labels, messages
static const uint8_t *kFontLarge = u8g2_font_helvR18_tf;  // card values

enum class Datum { TopLeft, MiddleCenter };

// Draws `text` positioned like TFT_eSPI's setTextDatum() did: TopLeft
// means (x,y) is the text's top-left corner, MiddleCenter means (x,y) is
// its center -- but via u8g2Fonts, so Swedish characters render
// correctly. `bg` must match whatever surface the text sits on (u8g2's
// "transparent" font mode did not reliably skip painting the glyph
// background in this setup, so every draw is explicitly opaque with a
// matching background color instead).
static void drawText(const String &text, int x, int y, const uint8_t *font, uint16_t color,
                      uint16_t bg, Datum datum = Datum::TopLeft) {
    u8g2Fonts.setFont(font);
    u8g2Fonts.setForegroundColor(color);
    u8g2Fonts.setBackgroundColor(bg);

    int ascent = u8g2Fonts.getFontAscent();
    int descent = u8g2Fonts.getFontDescent(); // negative

    int drawX = x, drawY = y;
    if (datum == Datum::TopLeft) {
        drawY = y + ascent;
    } else {
        int w = u8g2Fonts.getUTF8Width(text.c_str());
        drawX = x - w / 2;
        drawY = y + (ascent + descent) / 2;
    }
    u8g2Fonts.setCursor(drawX, drawY);
    u8g2Fonts.print(text);
}

static int textWidth(const char *text, const uint8_t *font) {
    u8g2Fonts.setFont(font);
    return u8g2Fonts.getUTF8Width(text);
}

static const uint32_t DISPLAY_CYCLE_MS = 5000;
static size_t g_displayIndex = 0;
static uint32_t g_displaySwitchedAtMs = 0;
static const uint32_t LOST_SIGNAL_AFTER_MS = 10000;

static bool g_graphMode = false;
static size_t g_graphIndex = 0;

static void drawCard(int x, int y, int w, int h, const char *label, const char *value, uint16_t accent) {
    const uint16_t cardBg = M5.Lcd.color565(15, 20, 30);
    const uint16_t cardBorder = M5.Lcd.color565(40, 55, 70);

    M5.Lcd.fillRoundRect(x, y, w, h, 6, cardBg);
    M5.Lcd.drawRoundRect(x, y, w, h, 6, cardBorder);

    drawText(label, x + 8, y + 4, kFontSmall, M5.Lcd.color565(140, 160, 180), cardBg);
    drawText(value, x + 8, y + 30, kFontLarge, accent, cardBg);
}

static void renderLive() {
    const AcInfinityReading &reading = g_readings[g_displayIndex];
    const KnownDevice &device = KNOWN_DEVICES[g_displayIndex];

    const uint16_t headerBg = M5.Lcd.color565(15, 25, 55);
    const uint16_t footerBg = M5.Lcd.color565(24, 24, 24);
    const uint16_t accentTeal = M5.Lcd.color565(0, 230, 200);
    const uint16_t accentOrange = M5.Lcd.color565(255, 170, 80);
    const uint16_t accentBlue = M5.Lcd.color565(120, 180, 255);
    const uint16_t accentGreen = M5.Lcd.color565(150, 220, 150);

    M5.Lcd.fillScreen(BLACK);

    // Header bar
    M5.Lcd.fillRect(0, 0, 320, 22, headerBg);
    drawText(device.name, 6, 4, kFontSmall, WHITE, headerBg);

    char posBuf[16];
    snprintf(posBuf, sizeof(posBuf), "(%u/%u)", (unsigned)(g_displayIndex + 1), (unsigned)KNOWN_DEVICE_COUNT);
    int posWidth = textWidth(posBuf, kFontSmall);
    drawText(posBuf, 320 - 6 - posWidth, 4, kFontSmall, M5.Lcd.color565(150, 170, 200), headerBg);

    if (!reading.valid) {
        const char *msg = "Waiting for signal...";
        int msgWidth = textWidth(msg, kFontSmall);
        drawText(msg, (320 - msgWidth) / 2, 100, kFontSmall, ORANGE, BLACK);
    } else if (millis() - reading.lastUpdateMs > LOST_SIGNAL_AFTER_MS) {
        const char *msg = "Lost signal, retrying...";
        int msgWidth = textWidth(msg, kFontSmall);
        drawText(msg, (320 - msgWidth) / 2, 100, kFontSmall, RED, BLACK);
    } else {
        uint32_t ageSec = (millis() - reading.lastUpdateMs) / 1000;
        char statusBuf[32];
        snprintf(statusBuf, sizeof(statusBuf), "LIVE - %lus ago - %d dBm", (unsigned long)ageSec, reading.rssi);
        drawText(statusBuf, 6, 25, kFontTiny, accentTeal, BLACK);

        char buf[32];
        const int margin = 8;
        const int tileW = (320 - margin * 3) / 2;
        const int tileH = 68;
        const int row1Y = 38;
        const int row2Y = row1Y + tileH + margin;
        const int col1X = margin;
        const int col2X = margin * 2 + tileW;

        snprintf(buf, sizeof(buf), "%.1f C", reading.temperatureC);
        drawCard(col1X, row1Y, tileW, tileH, "TEMPERATURE", buf, accentOrange);

        snprintf(buf, sizeof(buf), "%.1f %%", reading.humidityPct);
        drawCard(col2X, row1Y, tileW, tileH, "HUMIDITY", buf, accentBlue);

        snprintf(buf, sizeof(buf), "%.2f kPa", reading.vpdKpa);
        drawCard(col1X, row2Y, tileW, tileH, "VPD", buf, accentTeal);

        uint16_t batteryAccent = accentGreen;
        if (g_batteryValid[g_displayIndex]) {
            snprintf(buf, sizeof(buf), "%d %%", g_batteryPct[g_displayIndex]);
            batteryAccent = g_batteryPct[g_displayIndex] <= 20 ? ORANGE : accentGreen;
        } else {
            snprintf(buf, sizeof(buf), "--");
        }
        drawCard(col2X, row2Y, tileW, tileH, "BATTERY", buf, batteryAccent);
    }

    // Footer control bar
    M5.Lcd.fillRect(0, 220, 320, 20, footerBg);
    drawText("[A] Graph", 10, 224, kFontTiny, accentTeal, footerBg);
    drawText("[B/C] Select sensor", 140, 224, kFontTiny, accentTeal, footerBg);
}

static void renderGraph() {
    const History &h = g_history[g_graphIndex];
    const KnownDevice &device = KNOWN_DEVICES[g_graphIndex];

    const uint16_t headerBg = M5.Lcd.color565(15, 25, 55);
    const uint16_t gridColor = M5.Lcd.color565(45, 45, 45);
    const uint16_t fillColor = M5.Lcd.color565(0, 45, 50);
    const uint16_t lineColor = M5.Lcd.color565(0, 230, 200);
    const uint16_t footerBg = M5.Lcd.color565(24, 24, 24);

    M5.Lcd.fillScreen(BLACK);

    // Header bar
    M5.Lcd.fillRect(0, 0, 320, 22, headerBg);
    drawText(device.name, 6, 4, kFontSmall, WHITE, headerBg);

    char posBuf[16];
    snprintf(posBuf, sizeof(posBuf), "(%u/%u)", (unsigned)(g_graphIndex + 1), (unsigned)KNOWN_DEVICE_COUNT);
    int posWidth = textWidth(posBuf, kFontSmall);
    drawText(posBuf, 320 - 6 - posWidth, 4, kFontSmall, M5.Lcd.color565(150, 170, 200), headerBg);

    drawText("TEMPERATURE - LAST 24 HOURS", 6, 25, kFontTiny, lineColor, BLACK);

    const int plotX = 34, plotY = 40, plotW = 276, plotH = 130;
    const int plotBottom = plotY + plotH;

    if (h.count < 2) {
        M5.Lcd.drawRect(plotX, plotY, plotW, plotH, gridColor);
        const char *msg = "Not enough history yet";
        int msgWidth = textWidth(msg, kFontSmall);
        drawText(msg, plotX + (plotW - msgWidth) / 2, plotY + plotH / 2 - 8, kFontSmall, ORANGE, BLACK);
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

        // Gridlines -- 3 horizontal (quarter, half, three-quarter height) and
        // 3 vertical (6h/12h/18h marks), drawn before the data so the curve
        // and fill sit on top.
        for (int i = 1; i <= 3; i++) {
            int gy = plotY + plotH * i / 4;
            M5.Lcd.drawFastHLine(plotX, gy, plotW, gridColor);
        }
        for (int i = 1; i <= 3; i++) {
            int gx = plotX + plotW * i / 4;
            M5.Lcd.drawFastVLine(gx, plotY, plotH, gridColor);
        }
        M5.Lcd.drawRect(plotX, plotY, plotW, plotH, gridColor);

        auto valueToY = [&](float v) {
            return plotBottom - (int)((v - minT) / (maxT - minT) * plotH);
        };

        // Downsample to one point per pixel column so plotting stays fast
        // and legible regardless of how many samples are buffered (up to
        // HISTORY_SIZE for a full 24h at 1-minute resolution).
        int prevX = -1, prevY = -1, lastX = -1, lastY = -1;
        for (int col = 0; col < plotW; col++) {
            uint16_t sampleIdx = (uint32_t)col * (h.count - 1) / (plotW - 1);
            uint16_t idx = (h.head + HISTORY_SIZE - h.count + sampleIdx) % HISTORY_SIZE;
            float v = h.samples[idx];
            int x = plotX + col;
            int y = valueToY(v);

            // Filled area under the curve, drawn one column at a time.
            M5.Lcd.drawFastVLine(x, y, plotBottom - y, fillColor);

            if (prevX >= 0) {
                M5.Lcd.drawLine(prevX, prevY, x, y, lineColor);
                M5.Lcd.drawLine(prevX, prevY - 1, x, y - 1, lineColor);
            }
            prevX = x;
            prevY = y;
            lastX = x;
            lastY = y;
        }

        // Highlight the most recent reading.
        M5.Lcd.fillCircle(lastX, lastY, 3, WHITE);
        char nowBuf[16];
        snprintf(nowBuf, sizeof(nowBuf), "%.1fC", h.samples[(h.head + HISTORY_SIZE - 1) % HISTORY_SIZE]);
        int nowLabelWidth = textWidth(nowBuf, kFontTiny);
        int labelX = lastX - nowLabelWidth - 4;
        if (labelX < plotX) {
            labelX = lastX + 6;
        }
        int labelY = (lastY - plotY < 14) ? lastY + 4 : lastY - 14;
        drawText(nowBuf, labelX, labelY, kFontTiny, WHITE, BLACK);

        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", maxT);
        drawText(buf, 2, plotY - 2, kFontTiny, DARKGREY, BLACK);
        snprintf(buf, sizeof(buf), "%.1f", minT);
        drawText(buf, 2, plotBottom - 6, kFontTiny, DARKGREY, BLACK);
    }

    drawText("-24h", plotX, plotBottom + 4, kFontTiny, DARKGREY, BLACK);
    drawText("-12h", plotX + plotW / 2 - 8, plotBottom + 4, kFontTiny, DARKGREY, BLACK);
    const char *nowLabel = "now";
    int nowWidth = textWidth(nowLabel, kFontTiny);
    drawText(nowLabel, plotX + plotW - nowWidth, plotBottom + 4, kFontTiny, DARKGREY, BLACK);

    // Footer control bar
    M5.Lcd.fillRect(0, 220, 320, 20, footerBg);
    drawText("[A] Back", 10, 224, kFontTiny, M5.Lcd.color565(120, 220, 200), footerBg);
    drawText("[B] Prev device", 130, 224, kFontTiny, M5.Lcd.color565(120, 220, 200), footerBg);
    drawText("[C] Next device", 230, 224, kFontTiny, M5.Lcd.color565(120, 220, 200), footerBg);
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
// WiFi + OTA -- best-effort only. This device's job is BLE scanning and
// display, which need neither; a missing/failed WiFi connection just
// means no OTA updates until it's available, not an error.
// ---------------------------------------------------------------------
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 10000;
static const unsigned long WIFI_RETRY_INTERVAL_MS = 30000;
static bool g_otaReady = false;
static uint32_t g_lastWifiAttemptMs = 0;

static void otaBegin() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        Serial.println("OTA: update starting");
        M5.Lcd.fillScreen(BLACK);
        drawText("Uppdaterar firmware...", 10, 110, kFontSmall, WHITE, BLACK);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static int lastPercent = -1;
        int percent = total ? (int)((progress * 100UL) / total) : 0;
        if (percent == lastPercent) {
            return;
        }
        lastPercent = percent;
        char buf[24];
        snprintf(buf, sizeof(buf), "%d%%", percent);
        M5.Lcd.fillRect(0, 130, 320, 30, BLACK);
        drawText(buf, 10, 130, kFontLarge, WHITE, BLACK);
    });
    ArduinoOTA.onEnd([]() { Serial.println("OTA: update complete, rebooting"); });
    ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA: error %u\n", error); });

    ArduinoOTA.begin();
    g_otaReady = true;
    Serial.printf("OTA: ready as %s.local\n", OTA_HOSTNAME);
}

// Attempts a WiFi connection with a short timeout; never blocks for long
// and is safe to call repeatedly (e.g. from loop() if the link drops).
static void connectWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("WiFi: connecting to \"%s\"...\n", WIFI_SSID);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: connection failed (timeout) -- continuing without it");
        return;
    }
    Serial.printf("WiFi: connected, IP %s\n", WiFi.localIP().toString().c_str());
    if (!g_otaReady) {
        otaBegin();
    }
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

    u8g2Fonts.begin(gfxAdapter);
    u8g2Fonts.setFontDirection(0);

    g_sdReady = SD.cardType() != CARD_NONE;
    if (!g_sdReady) {
        // M5.begin() already tried SD.begin() at a fixed 40MHz, which some
        // cards/wiring can't sustain reliably (shows up as a low-level disk
        // I/O error even on a correctly FAT32-formatted card). Retry once
        // at the SD library's conservative default (4MHz) before giving up.
        Serial.println("Retrying SD card at a lower SPI speed...");
        SD.end();
        delay(50);
        g_sdReady = SD.begin(TFCARD_CS_PIN) && SD.cardType() != CARD_NONE;
    }
    if (g_sdReady) {
        Serial.println("SD card present -- history will persist across reboots");
        for (size_t i = 0; i < KNOWN_DEVICE_COUNT; i++) {
            loadHistoryFromSd(i);
        }
    } else {
        Serial.println("No SD card found -- history will be RAM-only this session");
    }

    M5.Lcd.fillScreen(BLACK);
    connectWifi(); // best-effort; BLE scanning below doesn't depend on it

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

    if (g_otaReady) {
        ArduinoOTA.handle();
    } else if (millis() - g_lastWifiAttemptMs > WIFI_RETRY_INTERVAL_MS) {
        g_lastWifiAttemptMs = millis();
        connectWifi();
    }

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

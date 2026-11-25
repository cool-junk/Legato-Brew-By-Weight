#include <Arduino.h>
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "TouchDrvCSTXXX.hpp"
#include "lv_conf.h"
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>
#include <ctype.h>

// Legato brew-by-weight controller: live DOT weight, touch UI, and relay outputs.
#define LEGATO_FW_VERSION "0.1.4"

#define LVGL_TICK_MS 2
// Small partial-render buffers keep LVGL in DMA-capable internal RAM; the
// extra rows provide safe headroom for the rotated partial-render path.
static constexpr uint32_t LVGL_DRAW_BUF_LINES = 40;
static constexpr uint32_t LVGL_DRAW_BUF_EXTRA_LINES = 4;
static constexpr uint8_t EVENT_WEIGHT = 1;
static constexpr uint8_t EVENT_BLE_UP = 2;
static constexpr uint8_t EVENT_BLE_DOWN = 3;
static constexpr uint8_t EVENT_BATTERY = 4;
static constexpr uint8_t COMMAND_TARE = 1;
static constexpr size_t MAX_PACKET = 32;
static constexpr uint8_t RELAY_MANUAL_PIN = 16;
static constexpr uint8_t RELAY_AUTO_PIN = 17;
static constexpr uint8_t RELAY_POWER_PIN = 18;
static constexpr uint16_t RELAY_PRESS_MS = 250;
static constexpr uint32_t BREW_MAX_RUNTIME_MS = 120000;
static constexpr float BREW_STOP_OFFSET_GRAMS = 0.0f;
// Tune this lead time to account for machine flow continuing after the stop press.
static constexpr uint16_t BREW_STOP_ANTICIPATION_DEFAULT_MS = 500;
static constexpr uint16_t BREW_STOP_ANTICIPATION_STEP_MS = 100;
static constexpr uint16_t BREW_STOP_ANTICIPATION_MAX_MS = 2000;
static constexpr float MAX_FLOW_RATE_GRAMS_PER_SECOND = 10.0f;
static constexpr float FLOW_RATE_FILTER_ALPHA = 0.35f;
static constexpr float BREW_START_MAX_WEIGHT_GRAMS = 5.0f;

enum BrewState : uint8_t { BREW_IDLE, BREW_RUNNING, BREW_COMPLETE, BREW_STOPPED, BREW_FAULT };

struct BrewEvent {
  uint8_t kind;
  float grams;
  uint8_t battery;
};
struct ScalePacket {
  uint8_t len;
  uint8_t data[MAX_PACKET];
};
struct BrewCommand { uint8_t kind; };

static QueueHandle_t appQueue = nullptr;
static QueueHandle_t blePacketQueue = nullptr;
static QueueHandle_t commandQueue = nullptr;
static NimBLEClient *dotClient = nullptr;
static String savedDotAddress;
static uint8_t savedDotAddressType = 0;
static volatile bool touchPending = false;
static bool lvTouchPressed = false;
static lv_coord_t lastTouchX = 0;
static lv_coord_t lastTouchY = 0;
static int16_t touchX[5], touchY[5];
static bool bleConnected = false;
static bool haveWeight = false;
static bool haveBattery = false;
static uint8_t batteryPercent = 0;
static bool combinedButtonPressed = false;
static bool combinedLongActionFired = false;
static uint32_t combinedButtonPressedAt = 0;
static bool modeButtonPressed = false;
static bool modeLongActionFired = false;
static uint32_t modeButtonPressedAt = 0;
static float liveGrams = 0.0f;
static uint16_t brewStopAnticipationMs = BREW_STOP_ANTICIPATION_DEFAULT_MS;
static float targetGrams = 36.0f;
static float espressoTargetGrams = 36.0f;
static float americanoTargetGrams = 150.0f;
static float beanTargetGrams = 18.0f;
enum BrewMode : uint8_t { MODE_ESPRESSO, MODE_AMERICANO, MODE_BEANS };
static BrewMode brewMode = MODE_ESPRESSO;
static bool infoPageActive = false;
static BrewMode infoReturnMode = MODE_ESPRESSO;
static uint32_t lastWeightAt = 0;
static float estimatedFlowRateGramsPerSecond = 0.0f;
static uint32_t lastFlowRateSampleAt = 0;
static bool haveFlowRateEstimate = false;
static BrewMode activeBrewMode = MODE_ESPRESSO;
static BrewState brewState = BREW_IDLE;
static uint32_t brewStartedAt = 0;
static const char *brewFaultText = "SCALE LOST - CHECK MACHINE";

static lv_disp_draw_buf_t draw_buf;
static uint32_t screenWidth = 0, screenHeight = 0;
static uint16_t *rotatedFlushBuffer = nullptr;
static size_t rotatedFlushBufferPixels = 0;
static Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
static Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 6, 0, 0, 0);
static TouchDrvCST92xx touch;

static lv_obj_t *progressArc = nullptr;
static lv_obj_t *modeButtonLabel = nullptr;
static lv_obj_t *statusLabel = nullptr;
static lv_obj_t *weightLabel = nullptr;
static lv_obj_t *weightUnitLabel = nullptr;
static lv_obj_t *targetLabel = nullptr;
static lv_obj_t *batteryLabel = nullptr;
static lv_obj_t *combinedButton = nullptr;
static lv_obj_t *combinedButtonLabel = nullptr;
static lv_obj_t *targetMinusButton = nullptr;
static lv_obj_t *targetPlusButton = nullptr;
static lv_obj_t *infoLabel = nullptr;
static lv_obj_t *infoAnticipationMinusButton = nullptr;
static lv_obj_t *infoAnticipationPlusButton = nullptr;

// LVGL's installed built-in Montserrat fonts stop at 48 px. Scale glyph
// bitmaps directly through the font callbacks to get a genuine 60 px layout
// without using LVGL's unsupported object transform path.
static constexpr uint8_t WEIGHT_FONT_SCALE_NUM = 3;
static constexpr uint8_t WEIGHT_FONT_SCALE_DEN = 2;
static uint8_t weightGlyphRawBitmap[96 * 96 / 2];
static uint8_t weightGlyphBitmap[96 * 96 / 2];
static lv_font_t weightFont;

static uint16_t scaleWeightMetric(uint16_t value) {
  return (uint16_t)((value * WEIGHT_FONT_SCALE_NUM + WEIGHT_FONT_SCALE_DEN / 2) /
                    WEIGHT_FONT_SCALE_DEN);
}

static bool weightFontGetGlyphDsc(const lv_font_t *, lv_font_glyph_dsc_t *dsc,
                                  uint32_t letter, uint32_t letterNext) {
  if (!lv_font_get_glyph_dsc(&lv_font_montserrat_48, dsc, letter, letterNext)) return false;
  dsc->adv_w = scaleWeightMetric(dsc->adv_w);
  dsc->box_w = scaleWeightMetric(dsc->box_w);
  dsc->box_h = scaleWeightMetric(dsc->box_h);
  dsc->ofs_x = (int16_t)(dsc->ofs_x * WEIGHT_FONT_SCALE_NUM / WEIGHT_FONT_SCALE_DEN);
  dsc->ofs_y = (int16_t)(dsc->ofs_y * WEIGHT_FONT_SCALE_NUM / WEIGHT_FONT_SCALE_DEN);
  if (dsc->box_w && dsc->box_h) {
    dsc->adv_w += 2;
    dsc->box_w += 2;
    dsc->box_h += 2;
    dsc->ofs_x -= 1;
    dsc->ofs_y -= 1;
  }
  dsc->bpp = 4;
  return true;
}

static uint8_t weightFontReadPixel(const uint8_t *bitmap, uint32_t index) {
  const uint8_t packed = bitmap[index >> 1];
  return (index & 1) ? (packed & 0x0F) : (packed >> 4);
}

static const uint8_t *weightFontGetGlyphBitmap(const lv_font_t *, uint32_t letter) {
  lv_font_glyph_dsc_t sourceDsc{};
  if (!lv_font_get_glyph_dsc(&lv_font_montserrat_48, &sourceDsc, letter, 0)) return nullptr;
  const uint8_t *source = lv_font_get_glyph_bitmap(&lv_font_montserrat_48, letter);
  if (!source || !sourceDsc.box_w || !sourceDsc.box_h) return source;

  const uint16_t scaledWidth = scaleWeightMetric(sourceDsc.box_w);
  const uint16_t scaledHeight = scaleWeightMetric(sourceDsc.box_h);
  const uint16_t outWidth = scaledWidth + 2;
  const uint16_t outHeight = scaledHeight + 2;
  if ((uint32_t)outWidth * outHeight > sizeof(weightGlyphBitmap) * 2U) return nullptr;
  memset(weightGlyphRawBitmap, 0, ((uint32_t)outWidth * outHeight + 1U) / 2U);
  memset(weightGlyphBitmap, 0, ((uint32_t)outWidth * outHeight + 1U) / 2U);

  for (uint16_t y = 0; y < scaledHeight; ++y) {
    int32_t yFixed = (int32_t)(((uint32_t)(2U * y + 1U) * WEIGHT_FONT_SCALE_DEN * 128U) /
                               WEIGHT_FONT_SCALE_NUM) - 128;
    if (yFixed < 0) yFixed = 0;
    const uint16_t y0 = (uint16_t)(yFixed >> 8);
    const uint16_t y1 = y0 + 1U < sourceDsc.box_h ? y0 + 1U : y0;
    const uint16_t fy = (uint16_t)(yFixed & 0xFF);
    for (uint16_t x = 0; x < scaledWidth; ++x) {
      int32_t xFixed = (int32_t)(((uint32_t)(2U * x + 1U) * WEIGHT_FONT_SCALE_DEN * 128U) /
                                 WEIGHT_FONT_SCALE_NUM) - 128;
      if (xFixed < 0) xFixed = 0;
      const uint16_t x0 = (uint16_t)(xFixed >> 8);
      const uint16_t x1 = x0 + 1U < sourceDsc.box_w ? x0 + 1U : x0;
      const uint16_t fx = (uint16_t)(xFixed & 0xFF);
      const uint8_t p00 = weightFontReadPixel(source, (uint32_t)y0 * sourceDsc.box_w + x0);
      const uint8_t p10 = weightFontReadPixel(source, (uint32_t)y0 * sourceDsc.box_w + x1);
      const uint8_t p01 = weightFontReadPixel(source, (uint32_t)y1 * sourceDsc.box_w + x0);
      const uint8_t p11 = weightFontReadPixel(source, (uint32_t)y1 * sourceDsc.box_w + x1);
      const uint16_t top = (uint16_t)(p00 * (256U - fx) + p10 * fx);
      const uint16_t bottom = (uint16_t)(p01 * (256U - fx) + p11 * fx);
      const uint8_t pixel = (uint8_t)((top * (256U - fy) + bottom * fy + 32768U) >> 16);
      const uint32_t index = (uint32_t)(y + 1U) * outWidth + (x + 1U);
      if (index & 1U) weightGlyphRawBitmap[index >> 1] |= pixel;
      else weightGlyphRawBitmap[index >> 1] |= (uint8_t)(pixel << 4);
    }
  }

  // A 3x3 max filter thickens the antialiased strokes by one pixel in every
  // direction while preserving a clean, antialiased edge.
  for (uint16_t y = 0; y < outHeight; ++y) {
    for (uint16_t x = 0; x < outWidth; ++x) {
      uint8_t boldPixel = 0;
      const uint16_t yStart = y ? y - 1U : y;
      const uint16_t yEnd = y + 1U < outHeight ? y + 1U : y;
      const uint16_t xStart = x ? x - 1U : x;
      const uint16_t xEnd = x + 1U < outWidth ? x + 1U : x;
      for (uint16_t sy = yStart; sy <= yEnd; ++sy) {
        for (uint16_t sx = xStart; sx <= xEnd; ++sx) {
          const uint8_t coverage = weightFontReadPixel(weightGlyphRawBitmap,
                                                       (uint32_t)sy * outWidth + sx);
          if (coverage > boldPixel) boldPixel = coverage;
        }
      }
      const uint32_t index = (uint32_t)y * outWidth + x;
      if (index & 1U) weightGlyphBitmap[index >> 1] |= boldPixel;
      else weightGlyphBitmap[index >> 1] |= (uint8_t)(boldPixel << 4);
    }
  }
  return weightGlyphBitmap;
}

static void initWeightFont() {
  memset(&weightFont, 0, sizeof(weightFont));
  weightFont.get_glyph_dsc = weightFontGetGlyphDsc;
  weightFont.get_glyph_bitmap = weightFontGetGlyphBitmap;
  weightFont.line_height = 80;
  weightFont.base_line = 14;
  weightFont.subpx = LV_FONT_SUBPX_NONE;
  weightFont.underline_position = -6;
  weightFont.underline_thickness = 3;
  weightFont.fallback = &lv_font_montserrat_48;
}

static void loadUiSettings() {
  Preferences prefs;
  if (prefs.begin("legato", true)) {
    const uint8_t storedMode = prefs.getUChar("mode", 0xFF);
    if (storedMode <= MODE_BEANS) brewMode = (BrewMode)storedMode;
    else brewMode = prefs.getBool("americano", false) ? MODE_AMERICANO : MODE_ESPRESSO;
    const bool hasOldTarget = prefs.isKey("target");
    const float oldTarget = prefs.getFloat("target", 0.0f);
    const uint16_t storedEspresso = prefs.getUShort("espresso_g", 0);
    const uint16_t storedAmericano = prefs.getUShort("americano_g", 0);
    const uint16_t storedBeans = prefs.getUShort("beans_g", 0);
    const uint16_t storedAnticipationMs = prefs.getUShort(
        "stop_ms", BREW_STOP_ANTICIPATION_DEFAULT_MS);
    prefs.end();

    if (storedAnticipationMs <= BREW_STOP_ANTICIPATION_MAX_MS &&
        storedAnticipationMs % BREW_STOP_ANTICIPATION_STEP_MS == 0) {
      brewStopAnticipationMs = storedAnticipationMs;
    }

    espressoTargetGrams = (storedEspresso >= 5 && storedEspresso <= 100)
                              ? storedEspresso
                              : (hasOldTarget && brewMode == MODE_ESPRESSO && oldTarget >= 5.0f && oldTarget <= 100.0f
                                     ? oldTarget : 36.0f);
    americanoTargetGrams = (storedAmericano >= 10 && storedAmericano <= 500)
                               ? storedAmericano
                               : (hasOldTarget && brewMode == MODE_AMERICANO && oldTarget >= 10.0f && oldTarget <= 500.0f
                                      ? oldTarget : 150.0f);
    beanTargetGrams = (storedBeans >= 1 && storedBeans <= 100) ? storedBeans : 18.0f;
  } else {
    Serial.println("Prefs: could not open settings namespace for reading; using defaults.");
  }
  targetGrams = brewMode == MODE_AMERICANO ? americanoTargetGrams
                 : brewMode == MODE_BEANS ? beanTargetGrams : espressoTargetGrams;
  const char *modeName = brewMode == MODE_AMERICANO ? "Americano"
                         : brewMode == MODE_BEANS ? "Beans" : "Espresso";
  Serial.printf("Prefs: loaded mode=%s, espresso=%ld g, americano=%ld g, beans=%ld g.\n",
                modeName, lroundf(espressoTargetGrams), lroundf(americanoTargetGrams),
                lroundf(beanTargetGrams));
}

static void saveUiSettings() {
  Preferences prefs;
  if (brewMode == MODE_AMERICANO) americanoTargetGrams = targetGrams;
  else if (brewMode == MODE_BEANS) beanTargetGrams = targetGrams;
  else espressoTargetGrams = targetGrams;

  if (!prefs.begin("legato", false)) {
    Serial.println("Prefs: could not open settings namespace for writing.");
    return;
  }
  prefs.putUChar("mode", brewMode);
  prefs.putBool("americano", brewMode == MODE_AMERICANO);
  prefs.putUShort("espresso_g", (uint16_t)lroundf(espressoTargetGrams));
  prefs.putUShort("americano_g", (uint16_t)lroundf(americanoTargetGrams));
  prefs.putUShort("beans_g", (uint16_t)lroundf(beanTargetGrams));
  prefs.putUShort("stop_ms", brewStopAnticipationMs);
  const bool verified = prefs.getUChar("mode", 0xFF) == brewMode &&
                        prefs.getBool("americano", brewMode != MODE_AMERICANO) == (brewMode == MODE_AMERICANO) &&
                        prefs.getUShort("espresso_g", 0) == (uint16_t)lroundf(espressoTargetGrams) &&
                        prefs.getUShort("americano_g", 0) == (uint16_t)lroundf(americanoTargetGrams) &&
                        prefs.getUShort("beans_g", 0) == (uint16_t)lroundf(beanTargetGrams) &&
                        prefs.getUShort("stop_ms", 0xFFFF) == brewStopAnticipationMs;
  prefs.end();
  Serial.printf("Prefs: settings save %s (Espresso %ld g, Americano %ld g, Beans %ld g, stop anticipation %u ms).\n",
                verified ? "verified" : "FAILED",
                lroundf(espressoTargetGrams), lroundf(americanoTargetGrams),
                lroundf(beanTargetGrams), (unsigned)brewStopAnticipationMs);
}

static uint16_t crc16Timemore(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
  }
  return crc;
}

static bool sendDotCommand(NimBLERemoteCharacteristic *commandChar,
                           uint8_t command, const uint8_t *payload, uint8_t payloadLen) {
  if (!commandChar || !commandChar->canWriteNoResponse()) return false;
  uint8_t frame[16] = {0xA5, 0x5A, 0x03, command, 0x00, payloadLen};
  if (payloadLen) memcpy(frame + 6, payload, payloadLen);
  uint16_t crc = crc16Timemore(frame, 6 + payloadLen);
  frame[6 + payloadLen] = (uint8_t)(crc >> 8);
  frame[7 + payloadLen] = (uint8_t)crc;
  return commandChar->writeValue(frame, 8 + payloadLen, false);
}

static bool requestDotBattery(NimBLERemoteCharacteristic *commandChar) {
  if (!commandChar || !commandChar->canWriteNoResponse()) return false;
  uint8_t frame[8] = {0xA5, 0x5A, 0x02, 0x05, 0x00, 0x00, 0x00, 0x00};
  const uint16_t crc = crc16Timemore(frame, 6);
  frame[6] = (uint8_t)(crc >> 8);
  frame[7] = (uint8_t)crc;
  return commandChar->writeValue(frame, sizeof(frame), false);
}

static void onDotNotify(NimBLERemoteCharacteristic *, uint8_t *data,
                        size_t length, bool) {
  if (!blePacketQueue || length < 8 || length > MAX_PACKET) return;
  ScalePacket packet{};
  packet.len = (uint8_t)length;
  memcpy(packet.data, data, length);
  xQueueSend(blePacketQueue, &packet, 0);
}

static bool isDot(const NimBLEAdvertisedDevice *device) {
  if (!device) return false;
  std::string name = device->haveName() ? device->getName() : std::string();
  for (char &ch : name) ch = (char)toupper((unsigned char)ch);
  return name.find("TIMEMORE") != std::string::npos ||
         name.find("DOT") != std::string::npos ||
         name.find("TES017") != std::string::npos ||
         device->isAdvertisingService(NimBLEUUID("FFF0"));
}

static void postAppEvent(uint8_t kind, float grams = 0.0f, uint8_t battery = 0) {
  if (!appQueue) return;
  BrewEvent event{kind, grams, battery};
  xQueueSend(appQueue, &event, 0);
}

static void loadSavedDot() {
  Preferences prefs;
  if (prefs.begin("legato", true)) {
    savedDotAddress = prefs.getString("dot_addr", "");
    savedDotAddressType = prefs.getUChar("dot_addr_type", 0);
    prefs.end();
  }
  if (savedDotAddress.length()) {
    Serial.printf("BLE: loaded saved DOT %s (type %u) from NVS.\n", savedDotAddress.c_str(), savedDotAddressType);
  } else {
    Serial.println("BLE: no saved DOT address in NVS.");
  }
}

static void saveDotAddress(const NimBLEAddress &address) {
  savedDotAddress = String(address.toString().c_str());
  savedDotAddressType = address.getType();
  Preferences prefs;
  bool verified = false;
  if (prefs.begin("legato", false)) {
    const size_t addressBytes = prefs.putString("dot_addr", savedDotAddress);
    prefs.putUChar("dot_addr_type", savedDotAddressType);
    verified = addressBytes > 0 && prefs.getString("dot_addr", "") == savedDotAddress &&
               prefs.getUChar("dot_addr_type", 0) == savedDotAddressType;
    prefs.end();
  }
  Serial.printf("BLE: NVS address save %s for %s (type %u).\n",
                verified ? "verified" : "FAILED", savedDotAddress.c_str(), savedDotAddressType);
}

static bool runDotSession(NimBLEClient *client) {
  if (blePacketQueue) xQueueReset(blePacketQueue);
  NimBLERemoteService *service = client->getService("FFF0");
  if (!service) {
    Serial.println("BLE: DOT service FFF0 missing.");
    return false;
  }
  NimBLERemoteCharacteristic *weightChar = service->getCharacteristic("FFF1");
  NimBLERemoteCharacteristic *commandChar = service->getCharacteristic("FFF2");
  if (!weightChar || !weightChar->canNotify() || !weightChar->subscribe(true, onDotNotify)) {
    Serial.println("BLE: could not subscribe to FFF1 notifications.");
    return false;
  }

  postAppEvent(EVENT_BLE_UP);
  Serial.println("BLE: connected and subscribed to DOT weight notifications.");
  bool batteryReceived = false;
  NimBLERemoteService *standardBatteryService = client->getService("180F");
  NimBLERemoteCharacteristic *standardBatteryChar = standardBatteryService
                                                       ? standardBatteryService->getCharacteristic("2A19")
                                                       : nullptr;
  if (standardBatteryChar && standardBatteryChar->canRead()) {
    NimBLEAttValue standardBattery = standardBatteryChar->readValue();
    if (standardBattery.size() == 1 && standardBattery[0] <= 100) {
      batteryReceived = true;
      postAppEvent(EVENT_BATTERY, 0.0f, standardBattery[0]);
      Serial.printf("DOT standard battery service: %u%%.\n", standardBattery[0]);
    } else {
      Serial.println("DOT standard battery characteristic returned no valid value.");
    }
  } else {
    Serial.println("DOT has no readable standard battery service; requesting its FFF0 battery report.");
  }
  delay(500);
  const uint8_t grams = 0x00;
  const uint8_t standardMode[2] = {0x01, 0x00};
  const uint8_t timerStart = 0x01;
  sendDotCommand(commandChar, 0x06, &grams, 1);
  delay(150);
  sendDotCommand(commandChar, 0x08, standardMode, 2);
  delay(150);
  sendDotCommand(commandChar, 0x02, &timerStart, 1);
  uint32_t lastBatteryRequestAt = 0;

  while (client->isConnected()) {
    BrewCommand command{};
    if (commandQueue && xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
      if (command.kind == COMMAND_TARE) {
        const bool sent = sendDotCommand(commandChar, 0x0D, nullptr, 0);
        Serial.printf("DOT tare command %s.\n", sent ? "sent" : "FAILED");
      }
    }
    const uint32_t batteryRetryMs = batteryReceived ? 60000 : 5000;
    if ((uint32_t)(millis() - lastBatteryRequestAt) >= batteryRetryMs || lastBatteryRequestAt == 0) {
      const bool requested = requestDotBattery(commandChar);
      Serial.printf("DOT battery request %s.\n", requested ? "sent" : "FAILED");
      lastBatteryRequestAt = millis();
    }
    ScalePacket packet{};
    if (xQueueReceive(blePacketQueue, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
      bool packetParsed = false;
      for (uint8_t offset = 0; offset + 8 <= packet.len; ++offset) {
        const uint8_t *p = packet.data + offset;
        if (p[0] != 0xA5 || p[1] != 0x5A) continue;
        const uint16_t payloadLen = ((uint16_t)p[4] << 8) | p[5];
        if (p[2] == 0x01 && p[3] == 0x01 && payloadLen == 9 && offset + 17 <= packet.len) {
          const uint32_t rawBits = ((uint32_t)p[6] << 24) | ((uint32_t)p[7] << 16) |
                                   ((uint32_t)p[8] << 8) | p[9];
          postAppEvent(EVENT_WEIGHT, (int32_t)rawBits / 10.0f);
          packetParsed = true;
          break;
        }
        if ((p[2] == 0x01 || p[2] == 0x02) && p[3] == 0x05 && payloadLen == 2 &&
            offset + 10 <= packet.len) {
          const uint8_t percent = p[7] > 100 ? 100 : p[7];
          postAppEvent(EVENT_BATTERY, 0.0f, percent);
          batteryReceived = true;
          packetParsed = true;
          Serial.printf("DOT battery: %u%% (%u bars).\n", percent, p[6]);
          break;
        }
      }
      if (!packetParsed) {
        // Keep every unrecognized notification visible while diagnosing optional
        // reports such as battery state. Some scale firmware may bundle frames.
        Serial.printf("DOT unparsed notification (%u bytes):", packet.len);
        for (uint8_t i = 0; i < packet.len; ++i) Serial.printf(" %02X", packet.data[i]);
        Serial.println();
      }
    }
  }
  postAppEvent(EVENT_BLE_DOWN);
  return true;
}

static void deleteDotClient() {
  if (!dotClient) return;
  if (dotClient->isConnected()) dotClient->disconnect();
  NimBLEDevice::deleteClient(dotClient);
  dotClient = nullptr;
}

static void bleTask(void *) {
  NimBLEDevice::init("Legato-Brew-Scale");
  // DOT normal advertising can be whitelist-only. Bond once while the scale
  // is in pairing mode, then NimBLE can reuse the stored keys after reboot.
  NimBLEDevice::setSecurityAuth(true, false, false);
  Serial.println("BLE: bonding enabled (Just Works, no MITM).");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(120);
  loadSavedDot();
  if (savedDotAddress.length()) {
    Serial.printf("BLE: saved DOT is %s (type %u).\n", savedDotAddress.c_str(), savedDotAddressType);
  }

  for (;;) {
    postAppEvent(EVENT_BLE_DOWN);
    // Connect to a currently advertising device. Some scale address types are
    // only connectable while the live advertisement is available, so do not
    // attempt a stale address-only connection from NVS.
    Serial.println("BLE: scanning for Timemore DOT...");
    NimBLEScanResults results = scan->getResults(8000);
    const NimBLEAdvertisedDevice *dot = nullptr;
    for (int i = 0; i < results.getCount(); ++i) {
      const NimBLEAdvertisedDevice *candidate = results.getDevice(i);
      if (!isDot(candidate)) continue;
      if (!dot) dot = candidate;
      if (savedDotAddress.length() &&
          String(candidate->getAddress().toString().c_str()) == savedDotAddress &&
          candidate->getAddress().getType() == savedDotAddressType) {
        dot = candidate;
        break;
      }
    }
    if (!dot) {
      scan->clearResults();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    const NimBLEAddress foundAddress = dot->getAddress();
    Serial.printf("BLE: selected DOT %s (type %u).\n", foundAddress.toString().c_str(), foundAddress.getType());
    dotClient = NimBLEDevice::createClient();
    if (!dotClient) {
      Serial.println("BLE: could not create client; retrying.");
      scan->clearResults();
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }
    dotClient->setConnectTimeout(5000);
    dotClient->setConnectRetries(7);
    if (dotClient->connect(dot)) {
      Serial.printf("BLE: connected to advertised DOT %s.\n", foundAddress.toString().c_str());
      saveDotAddress(foundAddress);
      if (dotClient->secureConnection()) {
        Serial.println("BLE: DOT link secured; bond is available for reconnects.");
      } else {
        Serial.println("BLE: security/bonding did not complete; put DOT in pairing mode for first bond.");
      }
      runDotSession(dotClient);
    } else {
      Serial.printf("BLE: connection failed after retries (NimBLE error %d); will scan again.\n",
                    dotClient->getLastError());
    }
    deleteDotClient();
    scan->clearResults();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void displayFlush(lv_disp_drv_t *display, const lv_area_t *area, lv_color_t *pixels) {
  const uint32_t sourceWidth = area->x2 - area->x1 + 1;
  const uint32_t sourceHeight = area->y2 - area->y1 + 1;
  const size_t tilePixels = (size_t)sourceWidth * sourceHeight;
  if (!rotatedFlushBuffer || tilePixels > rotatedFlushBufferPixels) {
    Serial.printf("Display rotation buffer too small (%u pixels needed, %u available).\n",
                  (unsigned)tilePixels, (unsigned)rotatedFlushBufferPixels);
    lv_disp_flush_ready(display);
    return;
  }

  // Match LVGL_DISP_ROT_270's logical-to-panel mapping: (x, y) -> (W - 1 - y, x).
  const int16_t destX = (int16_t)screenWidth - 1 - area->y2;
  const int16_t destY = area->x1;
  const uint32_t destWidth = sourceHeight;
  for (uint32_t sourceY = 0; sourceY < sourceHeight; ++sourceY) {
    for (uint32_t sourceX = 0; sourceX < sourceWidth; ++sourceX) {
      const uint32_t destPixelX = sourceHeight - 1 - sourceY;
      const uint32_t destPixelY = sourceX;
      const size_t sourceIndex = (size_t)sourceY * sourceWidth + sourceX;
      const size_t destIndex = (size_t)destPixelY * destWidth + destPixelX;
      rotatedFlushBuffer[destIndex] = pixels[sourceIndex].full;
    }
  }
#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(destX, destY, rotatedFlushBuffer, destWidth, sourceWidth);
#else
  gfx->draw16bitRGBBitmap(destX, destY, rotatedFlushBuffer, destWidth, sourceWidth);
#endif
  lv_disp_flush_ready(display);
}

static void lvglTick(void *) { lv_tick_inc(LVGL_TICK_MS); }

static void touchRead(lv_indev_drv_t *, lv_indev_data_t *data) {
  const bool touchInterruptPending = touchPending;
  data->state = lvTouchPressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  data->point.x = lastTouchX;
  data->point.y = lastTouchY;
  // Once a touch starts on the combined Start/Tare control, keep polling the
  // CST9217 until release so a one-second hold is measured reliably. The
  // controller may not return a fresh point on every poll, so retain the last
  // pressed state unless a touch interrupt reports an actual release.
  if (!touchInterruptPending && !combinedButtonPressed && !modeButtonPressed) return;
  const uint8_t points = touch.getPoint(touchX, touchY, touch.getSupportTouchPoint());
  touchPending = false;
  if (points) {
    // Inverse of the display-flush rotation: panel (x,y) -> logical (y,W-1-x).
    lastTouchX = touchY[0];
    lastTouchY = (lv_coord_t)screenHeight - 1 - touchX[0];
    lvTouchPressed = true;
    data->state = LV_INDEV_STATE_PR;
    data->point.x = lastTouchX;
    data->point.y = lastTouchY;
    Serial.printf("Touch: x=%d y=%d\n", touchX[0], touchY[0]);
  } else if (touchInterruptPending) {
    lvTouchPressed = false;
    data->state = LV_INDEV_STATE_REL;
  }
}

static void updateTargetLabel() {
  if (!targetLabel) return;
  const long targetWholeGrams = lroundf(targetGrams);
  lv_label_set_text_fmt(targetLabel, "TARGET  %ld g", targetWholeGrams);
}

static bool scaleDataFresh() {
  return bleConnected && haveWeight &&
         (uint32_t)(millis() - lastWeightAt) <= 2500;
}

static void relayPress(uint8_t pin, const char *name) {
  // TLP222A input LEDs are active-high. Keep all unselected channels LOW so
  // only one isolated switch contact can close at a time.
  digitalWrite(RELAY_MANUAL_PIN, LOW);
  digitalWrite(RELAY_AUTO_PIN, LOW);
  digitalWrite(RELAY_POWER_PIN, LOW);
  Serial.printf("Relay: press %s (%u ms).\n", name, RELAY_PRESS_MS);
  digitalWrite(pin, HIGH);
  delay(RELAY_PRESS_MS);
  digitalWrite(pin, LOW);
}

static void updateCombinedButtonUi() {
  if (!combinedButton || !combinedButtonLabel) return;
  const char *caption = nullptr;
  uint32_t color = 0x174A32;
  if (brewMode == MODE_BEANS) {
    caption = "TAP TO TARE\nBEANS ONLY";
    color = 0x202A2D;
  } else if (brewState == BREW_RUNNING) {
    caption = "TAP TO STOP";
    color = 0x7A2929;
  } else if (brewState == BREW_FAULT) {
    caption = "TAP TO TARE\nHOLD TO CLEAR FAULT";
    color = 0x7A2929;
  } else {
    caption = "TAP TO TARE\nHOLD TO START";
  }
  lv_label_set_text(combinedButtonLabel, caption);
  lv_obj_set_style_bg_color(combinedButton, lv_color_hex(color), 0);
}

static void stopBrew(BrewState result, const char *reason) {
  if (brewState != BREW_RUNNING) return;
  const uint8_t stopPin = activeBrewMode == MODE_AMERICANO
                              ? RELAY_MANUAL_PIN : RELAY_AUTO_PIN;
  const char *relayName = activeBrewMode == MODE_AMERICANO ? "Manual Brew stop"
                                                           : "Auto Brew stop";
  relayPress(stopPin, relayName);
  brewState = result;
  if (result == BREW_FAULT && reason) brewFaultText = reason;
  Serial.printf("Brew: %s\n", reason ? reason : "stopped");
  updateCombinedButtonUi();
}

static void startBrew() {
  if (brewMode == MODE_BEANS) {
    Serial.println("Brew ignored: Beans mode has no brew outputs.");
    return;
  }
  if (brewState == BREW_FAULT) {
    brewState = BREW_IDLE;
    Serial.println("Brew fault acknowledged. Confirm the machine is stopped before starting again.");
    updateCombinedButtonUi();
    return;
  }
  if (brewState == BREW_RUNNING) return;
  if (!scaleDataFresh()) {
    Serial.println("Brew start refused: DOT weight is not connected and fresh.");
    return;
  }
  if (liveGrams > BREW_START_MAX_WEIGHT_GRAMS) {
    Serial.printf("Brew start refused: scale reads %ld tenths g. Place the cup and tare before brewing.\n",
                  lroundf(liveGrams * 10.0f));
    return;
  }
  const float stopAt = targetGrams - BREW_STOP_OFFSET_GRAMS;
  if (liveGrams >= stopAt) {
    Serial.printf("Brew start refused: current weight %ld tenths g is at/above stop point %ld tenths g; tare first.\n",
                  lroundf(liveGrams * 10.0f), lroundf(stopAt * 10.0f));
    return;
  }

  activeBrewMode = brewMode;
  brewStartedAt = millis();
  estimatedFlowRateGramsPerSecond = 0.0f;
  lastFlowRateSampleAt = 0;
  haveFlowRateEstimate = false;
  brewState = BREW_RUNNING;
  if (activeBrewMode == MODE_AMERICANO) relayPress(RELAY_MANUAL_PIN, "Manual Brew start");
  else relayPress(RELAY_AUTO_PIN, "Auto Brew start");
  Serial.printf("Brew: started %s at %ld tenths g; stop target %ld tenths g; max runtime %lu s.\n",
                activeBrewMode == MODE_AMERICANO ? "Americano" : "Espresso",
                lroundf(liveGrams * 10.0f), lroundf(stopAt * 10.0f),
                (unsigned long)(BREW_MAX_RUNTIME_MS / 1000));
  updateCombinedButtonUi();
}

static void requestScaleTare() {
  if (brewState == BREW_RUNNING) {
    Serial.println("DOT tare ignored while brewing; hold the button to stop.");
    return;
  }
  if (!bleConnected || !commandQueue) {
    Serial.println("DOT tare ignored: scale is not connected.");
    return;
  }
  BrewCommand command{COMMAND_TARE};
  if (xQueueSend(commandQueue, &command, 0) != pdTRUE) {
    Serial.println("DOT tare ignored: command queue is full.");
    return;
  }
  Serial.println("DOT tare requested; display will update when the scale reports the tared reading.");
}

static void performCombinedLongAction() {
  if (brewMode == MODE_BEANS) return;
  if (brewState == BREW_RUNNING) return;
  startBrew();
}

static void onCombinedButton(lv_event_t *event) {
  const lv_event_code_t code = lv_event_get_code(event);
  if (code == LV_EVENT_PRESSED) {
    combinedButtonPressed = true;
    combinedLongActionFired = false;
    combinedButtonPressedAt = millis();
  } else if (code == LV_EVENT_RELEASED) {
    if (!combinedButtonPressed) return;
    const uint32_t heldMs = millis() - combinedButtonPressedAt;
    combinedButtonPressed = false;
    if (combinedLongActionFired) return;
    if (brewState == BREW_RUNNING) stopBrew(BREW_STOPPED, "user tap-to-stop requested");
    else if (brewMode != MODE_BEANS && heldMs >= 1000) performCombinedLongAction();
    else requestScaleTare();
  } else if (code == LV_EVENT_PRESS_LOST) {
    combinedButtonPressed = false;
    combinedLongActionFired = false;
  }
}

static void updateCombinedButtonHold() {
  if (!combinedButtonPressed || combinedLongActionFired ||
      brewMode == MODE_BEANS || brewState == BREW_RUNNING) return;
  if ((uint32_t)(millis() - combinedButtonPressedAt) >= 1000) {
    combinedLongActionFired = true;
    performCombinedLongAction();
  }
}

static void onPowerButton(lv_event_t *) {
  if (brewState == BREW_RUNNING) {
    Serial.println("Power press ignored while brewing; stop the active brew first.");
    return;
  }
  relayPress(RELAY_POWER_PIN, "Power");
}

static void updateBrewSafety() {
  if (brewState != BREW_RUNNING) return;
  if (!scaleDataFresh()) {
    stopBrew(BREW_FAULT, "scale disconnected or stale; stop pulse sent - check machine");
    return;
  }
  if ((uint32_t)(millis() - brewStartedAt) >= BREW_MAX_RUNTIME_MS) {
    stopBrew(BREW_FAULT, "maximum brew time reached; stop pulse sent - check machine");
    return;
  }
  float predictedWeight = liveGrams;
  if (haveFlowRateEstimate &&
      (uint32_t)(millis() - lastFlowRateSampleAt) <= 1000) {
    predictedWeight += estimatedFlowRateGramsPerSecond *
                       (brewStopAnticipationMs / 1000.0f);
  }
  const float stopAt = targetGrams - BREW_STOP_OFFSET_GRAMS;
  if (predictedWeight >= stopAt) {
    stopBrew(BREW_COMPLETE, "predicted target reached; stop pulse sent");
  }
}

static void updateModeLabel() {
  if (!modeButtonLabel) return;
  const char *label = infoPageActive ? "INFO"
                      : brewMode == MODE_AMERICANO ? "AMERICANO"
                      : brewMode == MODE_BEANS ? "BEANS" : "ESPRESSO";
  lv_label_set_text(modeButtonLabel, label);
  updateTargetLabel();
  updateCombinedButtonUi();
  lv_obj_t *pageObjects[] = {
      progressArc, statusLabel, weightLabel, weightUnitLabel, targetLabel,
      batteryLabel, combinedButton, targetMinusButton, targetPlusButton};
  for (lv_obj_t *object : pageObjects) {
    if (!object) continue;
    if (infoPageActive) lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(object, LV_OBJ_FLAG_HIDDEN);
  }
  if (infoLabel) {
    if (infoPageActive) lv_obj_clear_flag(infoLabel, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(infoLabel, LV_OBJ_FLAG_HIDDEN);
  }
  lv_obj_t *infoControls[] = {infoAnticipationMinusButton, infoAnticipationPlusButton};
  for (lv_obj_t *control : infoControls) {
    if (!control) continue;
    if (infoPageActive) lv_obj_clear_flag(control, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(control, LV_OBJ_FLAG_HIDDEN);
  }
}

static void updateInfoPage() {
  if (!infoLabel || !infoPageActive) return;
  const char *mode = brewMode == MODE_AMERICANO ? "Americano"
                     : brewMode == MODE_BEANS ? "Beans" : "Espresso";
  const char *scaleStatus = !bleConnected ? "Offline"
                            : scaleDataFresh() ? "Live" : "Waiting for weight";
  const char *brewStatus = brewState == BREW_RUNNING ? "Running"
                           : brewState == BREW_COMPLETE ? "Target reached"
                           : brewState == BREW_STOPPED ? "Stopped"
                           : brewState == BREW_FAULT ? "FAULT - inspect machine"
                           : "Idle";
  char weightText[20];
  if (haveWeight) {
    const int32_t tenths = (int32_t)lroundf(liveGrams * 10.0f);
    snprintf(weightText, sizeof(weightText), "%ld.%01ld g",
             (long)(tenths / 10), (long)labs(tenths % 10));
  } else {
    snprintf(weightText, sizeof(weightText), "--.- g");
  }
  char batteryText[12];
  if (haveBattery) snprintf(batteryText, sizeof(batteryText), "%u%%", batteryPercent);
  else snprintf(batteryText, sizeof(batteryText), "Unknown");
  const uint32_t uptimeSeconds = millis() / 1000;
  lv_label_set_text_fmt(infoLabel,
      "Legato Brew-by-Weight  v%s\n"
      "Build: %s %s\n"
      "\n"
      "Mode: %s    Brew: %s\n"
      "Target: %ld g    Weight: %s\n"
      "Scale: %s    DOT battery: %s\n"
      "TLP222A pulse: %u ms (active-high)\n"
      "Manual GPIO%u   Auto GPIO%u   Power GPIO%u\n"
      "Stop anticipation: %u ms\n"
      "Flow estimate: %ld.%ld g/s\n"
      "Max brew: %lu s   Scale-loss stop: 2.5 s\n"
      "Start limit: %d g   Stop offset: %d g\n"
      "Uptime: %lu h %lu min\n"
      "Tap - / + below to adjust by 100 ms\n"
      "Tap mode to return; hold 3 s for Info",
      LEGATO_FW_VERSION, __DATE__, __TIME__, mode, brewStatus,
      lroundf(targetGrams), weightText, scaleStatus, batteryText,
      RELAY_PRESS_MS, RELAY_MANUAL_PIN, RELAY_AUTO_PIN, RELAY_POWER_PIN,
      (unsigned)brewStopAnticipationMs,
      (long)(lroundf(estimatedFlowRateGramsPerSecond * 10.0f) / 10),
      (long)(labs(lroundf(estimatedFlowRateGramsPerSecond * 10.0f)) % 10),
      (unsigned long)(BREW_MAX_RUNTIME_MS / 1000),
      (int)BREW_START_MAX_WEIGHT_GRAMS, (int)BREW_STOP_OFFSET_GRAMS,
      (unsigned long)(uptimeSeconds / 3600),
      (unsigned long)((uptimeSeconds / 60) % 60));
}

static void onAnticipationMinus(lv_event_t *) {
  if (!infoPageActive || brewStopAnticipationMs == 0) return;
  if (brewStopAnticipationMs >= BREW_STOP_ANTICIPATION_STEP_MS) {
    brewStopAnticipationMs -= BREW_STOP_ANTICIPATION_STEP_MS;
  } else {
    brewStopAnticipationMs = 0;
  }
  saveUiSettings();
  updateInfoPage();
  Serial.printf("Brew stop anticipation set to %u ms.\n",
                (unsigned)brewStopAnticipationMs);
}

static void onAnticipationPlus(lv_event_t *) {
  if (!infoPageActive ||
      brewStopAnticipationMs >= BREW_STOP_ANTICIPATION_MAX_MS) return;
  brewStopAnticipationMs += BREW_STOP_ANTICIPATION_STEP_MS;
  if (brewStopAnticipationMs > BREW_STOP_ANTICIPATION_MAX_MS) {
    brewStopAnticipationMs = BREW_STOP_ANTICIPATION_MAX_MS;
  }
  saveUiSettings();
  updateInfoPage();
  Serial.printf("Brew stop anticipation set to %u ms.\n",
                (unsigned)brewStopAnticipationMs);
}

static void updateDisplayValues() {
  if (weightLabel) {
    if (haveWeight) {
      // Use integer formatting here: LVGL's built-in formatter may omit %f.
      const int32_t tenths = (int32_t)lroundf(liveGrams * 10.0f);
      const bool negative = tenths < 0;
      const uint32_t magnitude = negative ? (uint32_t)(-(int64_t)tenths) : (uint32_t)tenths;
      const uint32_t whole = magnitude / 10;
      const uint32_t decimal = magnitude % 10;
      if (negative) lv_label_set_text_fmt(weightLabel, "-%lu.%lu", (unsigned long)whole, (unsigned long)decimal);
      else lv_label_set_text_fmt(weightLabel, "%lu.%lu", (unsigned long)whole, (unsigned long)decimal);
    } else {
      lv_label_set_text(weightLabel, "--.-");
    }
  }
  if (progressArc) {
    int value = 0;
    if (haveWeight && targetGrams > 0.0f) {
      float ratio = liveGrams / targetGrams;
      if (ratio < 0.0f) ratio = 0.0f;
      if (ratio > 1.0f) ratio = 1.0f;
      value = (int)(ratio * 1000.0f);
    }
    lv_arc_set_value(progressArc, value);
  }
}

static void onTargetMinus(lv_event_t *) {
  if (brewState == BREW_RUNNING) return;
  const float step = brewMode == MODE_AMERICANO ? 5.0f : 1.0f;
  const float minimum = brewMode == MODE_AMERICANO ? 10.0f : 1.0f;
  targetGrams -= step;
  if (targetGrams < minimum) targetGrams = minimum;
  saveUiSettings();
  updateTargetLabel();
  updateDisplayValues();
  Serial.printf("Target decreased to %ld g\n", lroundf(targetGrams));
}
static void onTargetPlus(lv_event_t *) {
  if (brewState == BREW_RUNNING) return;
  const float step = brewMode == MODE_AMERICANO ? 5.0f : 1.0f;
  const float maximum = brewMode == MODE_AMERICANO ? 500.0f : 100.0f;
  targetGrams += step;
  if (targetGrams > maximum) targetGrams = maximum;
  saveUiSettings();
  updateTargetLabel();
  updateDisplayValues();
  Serial.printf("Target increased to %ld g\n", lroundf(targetGrams));
}
static void onModeToggle(lv_event_t *) {
  if (modeLongActionFired) {
    modeLongActionFired = false;
    return;
  }
  if (infoPageActive) {
    infoPageActive = false;
    brewMode = infoReturnMode;
  } else {
    if (brewState == BREW_RUNNING || brewState == BREW_FAULT) {
      Serial.println("Mode changes are disabled while brewing or a fault is latched.");
      return;
    }
    brewMode = (BrewMode)(brewMode + 1);
    if (brewMode > MODE_BEANS) brewMode = MODE_ESPRESSO;
  }
  if (!infoPageActive &&
      (brewState == BREW_COMPLETE || brewState == BREW_STOPPED)) {
    brewState = BREW_IDLE;
  }
  if (!infoPageActive) {
    targetGrams = brewMode == MODE_AMERICANO ? americanoTargetGrams
                   : brewMode == MODE_BEANS ? beanTargetGrams : espressoTargetGrams;
  }
  saveUiSettings();
  updateModeLabel();
  updateDisplayValues();
  updateInfoPage();
}

static void toggleInfoPage() {
  if (!infoPageActive && brewState == BREW_RUNNING) {
    Serial.println("Info page is unavailable while brewing; stop the brew first.");
    return;
  }
  if (infoPageActive) {
    infoPageActive = false;
    brewMode = infoReturnMode;
  } else {
    infoReturnMode = brewMode;
    infoPageActive = true;
  }
  updateModeLabel();
  updateDisplayValues();
  updateInfoPage();
}

static void onModeButtonGesture(lv_event_t *event) {
  const lv_event_code_t code = lv_event_get_code(event);
  if (code == LV_EVENT_PRESSED) {
    modeButtonPressed = true;
    modeLongActionFired = false;
    modeButtonPressedAt = millis();
  } else if (code == LV_EVENT_RELEASED) {
    if (!modeButtonPressed) return;
    const uint32_t heldMs = millis() - modeButtonPressedAt;
    modeButtonPressed = false;
    if (!modeLongActionFired && heldMs >= 3000) {
      modeLongActionFired = true;
      toggleInfoPage();
    }
  } else if (code == LV_EVENT_PRESS_LOST) {
    modeButtonPressed = false;
    modeLongActionFired = false;
  }
}

static void updateModeButtonHold() {
  if (!modeButtonPressed || modeLongActionFired) return;
  if ((uint32_t)(millis() - modeButtonPressedAt) >= 3000) {
    modeLongActionFired = true;
    toggleInfoPage();
  }
}

static void displayRounder(lv_disp_drv_t *, lv_area_t *area) {
  // CO5300/QSPI flushes work reliably when the draw window starts on even pixels.
  if (area->x1 & 1) area->x1--;
  if (area->y1 & 1) area->y1--;
  if ((area->x2 & 1) == 0 && area->x2 < (lv_coord_t)screenWidth - 1) area->x2++;
  if ((area->y2 & 1) == 0 && area->y2 < (lv_coord_t)screenHeight - 1) area->y2++;
}

static lv_obj_t *makeButton(lv_obj_t *parent, const char *caption, lv_coord_t x,
                            lv_coord_t y, lv_coord_t w, lv_coord_t h,
                            lv_event_cb_t callback, const lv_font_t *font,
                            lv_obj_t **labelOut = nullptr) {
  lv_obj_t *button = lv_btn_create(parent);
  lv_obj_set_pos(button, x, y);
  lv_obj_set_size(button, w, h);
  lv_obj_set_style_radius(button, 20, 0);
  lv_obj_set_style_bg_color(button, lv_color_hex(0x202A2D), 0);
  lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(button, 1, 0);
  lv_obj_set_style_border_color(button, lv_color_hex(0x3A484B), 0);
  lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *label = lv_label_create(button);
  lv_label_set_text(label, caption);
  lv_obj_center(label);
  lv_obj_set_style_text_color(label, lv_color_hex(0xE9F1EE), 0);
  lv_obj_set_style_text_font(label, font, 0);
  if (labelOut) *labelOut = label;
  return button;
}

static void createUi() {
  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x080D0F), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

  progressArc = lv_arc_create(screen);
  lv_obj_set_size(progressArc, 438, 438);
  lv_obj_center(progressArc);
  lv_arc_set_rotation(progressArc, 270);
  // Leave a top gap across the Power button. At this radius, 14 degrees on
  // each side aligns the progress arc endpoints with the button's edges.
  lv_arc_set_bg_angles(progressArc, 14, 346);
  lv_arc_set_range(progressArc, 0, 1000);
  lv_arc_set_value(progressArc, 0);
  lv_obj_remove_style(progressArc, nullptr, LV_PART_KNOB);
  lv_obj_set_style_bg_opa(progressArc, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_arc_width(progressArc, 9, LV_PART_MAIN);
  lv_obj_set_style_arc_color(progressArc, lv_color_hex(0x1B302A), LV_PART_MAIN);
  lv_obj_set_style_arc_opa(progressArc, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_arc_width(progressArc, 9, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(progressArc, lv_color_hex(0x4FE08A), LV_PART_INDICATOR);
  lv_obj_set_style_arc_opa(progressArc, LV_OPA_COVER, LV_PART_INDICATOR);
  // The arc shows progress only; it must not respond to touch or dragging.
  lv_obj_clear_flag(progressArc, LV_OBJ_FLAG_CLICKABLE);

  makeButton(screen, "POWER", 183, 8, 100, 36, onPowerButton, &lv_font_montserrat_18);

  lv_obj_t *modeButton = lv_btn_create(screen);
  lv_obj_set_size(modeButton, 200, 54);
  lv_obj_align(modeButton, LV_ALIGN_TOP_MID, 0, 50);
  lv_obj_set_style_radius(modeButton, 26, 0);
  lv_obj_set_style_bg_color(modeButton, lv_color_hex(0x182321), 0);
  lv_obj_set_style_border_width(modeButton, 1, 0);
  lv_obj_set_style_border_color(modeButton, lv_color_hex(0x345344), 0);
  lv_obj_add_event_cb(modeButton, onModeToggle, LV_EVENT_CLICKED, nullptr);
  lv_obj_add_event_cb(modeButton, onModeButtonGesture, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(modeButton, onModeButtonGesture, LV_EVENT_RELEASED, nullptr);
  lv_obj_add_event_cb(modeButton, onModeButtonGesture, LV_EVENT_PRESS_LOST, nullptr);
  modeButtonLabel = lv_label_create(modeButton);
  lv_obj_set_style_text_color(modeButtonLabel, lv_color_hex(0x77E6A2), 0);
  lv_obj_set_style_text_font(modeButtonLabel, &lv_font_montserrat_24, 0);
  lv_obj_center(modeButtonLabel);
  updateModeLabel();

  statusLabel = lv_label_create(screen);
  lv_obj_align(statusLabel, LV_ALIGN_TOP_MID, 0, 106);
  lv_obj_set_style_text_color(statusLabel, lv_color_hex(0x98A7A3), 0);
  lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_16, 0);
  lv_label_set_text(statusLabel, "SEARCHING FOR DOT");

  infoLabel = lv_label_create(screen);
  lv_obj_set_width(infoLabel, 400);
  lv_obj_set_height(infoLabel, 286);
  lv_obj_align(infoLabel, LV_ALIGN_CENTER, 0, 24);
  lv_obj_set_style_text_color(infoLabel, lv_color_hex(0xD7E2DE), 0);
  lv_obj_set_style_text_font(infoLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_align(infoLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_label_set_long_mode(infoLabel, LV_LABEL_LONG_WRAP);
  lv_obj_add_flag(infoLabel, LV_OBJ_FLAG_HIDDEN);
  infoAnticipationMinusButton = makeButton(screen, "-100 ms", 63, 404, 150, 44,
                                           onAnticipationMinus, &lv_font_montserrat_16);
  infoAnticipationPlusButton = makeButton(screen, "+100 ms", 253, 404, 150, 44,
                                          onAnticipationPlus, &lv_font_montserrat_16);
  lv_obj_add_flag(infoAnticipationMinusButton, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(infoAnticipationPlusButton, LV_OBJ_FLAG_HIDDEN);

  weightLabel = lv_label_create(screen);
  lv_obj_set_width(weightLabel, 260);
  // Anchor every reading to the same right edge, regardless of its length.
  lv_obj_align(weightLabel, LV_ALIGN_CENTER, -60, -63);
  lv_obj_set_style_text_align(weightLabel, LV_TEXT_ALIGN_RIGHT, 0);
  lv_obj_set_style_text_color(weightLabel, lv_color_hex(0xF4F7F6), 0);
  lv_obj_set_style_text_font(weightLabel, &weightFont, 0);
  lv_label_set_text(weightLabel, "--.-");

  weightUnitLabel = lv_label_create(screen);
  lv_label_set_text(weightUnitLabel, "g");
  lv_obj_align(weightUnitLabel, LV_ALIGN_CENTER, 100, -63);
  lv_obj_set_style_text_color(weightUnitLabel, lv_color_hex(0x9FAEAA), 0);
  lv_obj_set_style_text_font(weightUnitLabel, &lv_font_montserrat_20, 0);

  targetLabel = lv_label_create(screen);
  lv_obj_align(targetLabel, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_color(targetLabel, lv_color_hex(0xC6D1CD), 0);
  lv_obj_set_style_text_font(targetLabel, &lv_font_montserrat_24, 0);
  updateTargetLabel();

  batteryLabel = lv_label_create(screen);
  // Keep the battery indicator below the main control and inside the perimeter ring.
  lv_obj_align(batteryLabel, LV_ALIGN_BOTTOM_MID, 0, -36);
  lv_obj_set_style_text_color(batteryLabel, lv_color_hex(0x77E6A2), 0);
  lv_obj_set_style_text_font(batteryLabel, &lv_font_montserrat_16, 0);
  lv_label_set_text(batteryLabel, "BAT --%");

  combinedButton = makeButton(screen, "TAP TO TARE\nHOLD TO START",
                              43, 246, 380, 108, onCombinedButton,
                              &lv_font_montserrat_32, &combinedButtonLabel);
  lv_obj_set_width(combinedButtonLabel, LV_PCT(100));
  lv_obj_set_style_text_align(combinedButtonLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(combinedButtonLabel);
  lv_obj_add_event_cb(combinedButton, onCombinedButton, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(combinedButton, onCombinedButton, LV_EVENT_RELEASED, nullptr);
  lv_obj_add_event_cb(combinedButton, onCombinedButton, LV_EVENT_PRESS_LOST, nullptr);
  targetMinusButton = makeButton(screen, "-", 116, 360, 60, 40,
                                 onTargetMinus, &lv_font_montserrat_24);
  targetPlusButton = makeButton(screen, "+", 290, 360, 60, 40,
                                onTargetPlus, &lv_font_montserrat_24);
  updateModeLabel();
  updateInfoPage();
  updateDisplayValues();
}

static void updateStatusLabel() {
  if (!statusLabel) return;
  const char *text = "";
  uint32_t color = 0x98A7A3;
  if (brewState == BREW_FAULT) {
    text = brewFaultText;
    color = 0xFF4D4D;
  } else if (brewState == BREW_RUNNING) {
    text = activeBrewMode == MODE_AMERICANO ? "AMERICANO WATER DISPENSING"
                                             : "ESPRESSO BREWING";
    color = 0x66D992;
  } else if (brewState == BREW_COMPLETE) {
    text = "TARGET REACHED - STOP SENT";
    color = 0x66D992;
  } else if (brewState == BREW_STOPPED) {
    text = "BREW STOPPED";
    color = 0xE4B866;
  } else if (brewMode == MODE_BEANS) {
    text = scaleDataFresh() ? "BEAN WEIGHING - OUTPUTS OFF"
                            : (bleConnected ? "WAITING FOR DOT WEIGHT" : "BEAN MODE - DOT SEARCHING");
    color = scaleDataFresh() ? 0x66D992 : 0xE4B866;
  } else if (!bleConnected) {
    text = "SEARCHING FOR DOT";
    color = 0xE4B866;
  } else if (!scaleDataFresh()) {
    text = "CONNECTED - WAITING FOR WEIGHT";
    color = 0xE4B866;
  } else {
    text = "DOT CONNECTED - LIVE";
    color = 0x66D992;
  }

  static const char *shownText = nullptr;
  if (!shownText || strcmp(shownText, text) != 0) {
    shownText = text;
    lv_label_set_text(statusLabel, text);
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(color), 0);
  }
}

static void processAppEvents() {
  bool displayChanged = false;
  BrewEvent event{};
  while (xQueueReceive(appQueue, &event, 0) == pdTRUE) {
    if (event.kind == EVENT_BLE_UP) {
      bleConnected = true;
      Serial.println("UI: DOT connected.");
    } else if (event.kind == EVENT_BLE_DOWN) {
      bleConnected = false;
      haveWeight = false;
      haveBattery = false;
      if (batteryLabel) {
        lv_label_set_text(batteryLabel, "BAT --%");
        lv_obj_set_style_text_color(batteryLabel, lv_color_hex(0x77E6A2), 0);
      }
      Serial.println("UI: DOT disconnected/searching.");
      displayChanged = true;
    } else if (event.kind == EVENT_WEIGHT) {
      const uint32_t weightSampleAt = millis();
      if (brewState == BREW_RUNNING && haveWeight) {
        const uint32_t sampleIntervalMs = weightSampleAt - lastWeightAt;
        if (sampleIntervalMs > 0 && sampleIntervalMs <= 2500) {
          float sampleRate = (event.grams - liveGrams) * 1000.0f /
                             sampleIntervalMs;
          if (sampleRate < 0.0f) sampleRate = 0.0f;
          if (sampleRate > MAX_FLOW_RATE_GRAMS_PER_SECOND) {
            sampleRate = MAX_FLOW_RATE_GRAMS_PER_SECOND;
          }
          if (haveFlowRateEstimate) {
            estimatedFlowRateGramsPerSecond =
                FLOW_RATE_FILTER_ALPHA * sampleRate +
                (1.0f - FLOW_RATE_FILTER_ALPHA) * estimatedFlowRateGramsPerSecond;
          } else {
            estimatedFlowRateGramsPerSecond = sampleRate;
            haveFlowRateEstimate = true;
          }
          lastFlowRateSampleAt = weightSampleAt;
        }
      }
      bleConnected = true;
      haveWeight = true;
      liveGrams = event.grams;
      lastWeightAt = weightSampleAt;
      displayChanged = true;
      const int32_t tenths = (int32_t)lroundf(liveGrams * 10.0f);
      Serial.printf("DOT weight: %ld.%01ld g\n", (long)(tenths / 10),
                    (long)labs(tenths % 10));
    } else if (event.kind == EVENT_BATTERY) {
      haveBattery = true;
      batteryPercent = event.battery;
      if (batteryLabel) {
        lv_label_set_text_fmt(batteryLabel, "BAT %u%%", batteryPercent);
        lv_obj_set_style_text_color(batteryLabel,
                                    lv_color_hex(batteryPercent < 20 ? 0xFF4D4D : 0x77E6A2), 0);
        lv_obj_invalidate(batteryLabel);
      }
      Serial.printf("UI: battery indicator updated to %u%%.\n", batteryPercent);
    }
  }
  static uint32_t lastStatusRefresh = 0;
  if ((uint32_t)(millis() - lastStatusRefresh) >= 250) {
    lastStatusRefresh = millis();
    updateStatusLabel();
  }
  if (displayChanged) updateDisplayValues();
}

static void initRelayOutputs() {
  const uint8_t pins[] = {RELAY_MANUAL_PIN, RELAY_AUTO_PIN, RELAY_POWER_PIN};
  for (uint8_t pin : pins) {
    // TLP222A LED inputs turn on at HIGH. Preload LOW before output mode so
    // boot starts with all three PhotoMOS contacts open.
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

void setup() {
  initRelayOutputs();
  Serial.begin(115200);
  delay(400);
  Serial.println("Legato brew-by-weight controller. Active-high TLP222A outputs initialized OFF.");
  Serial.println("PhotoMOS map: GPIO16 Manual, GPIO17 Auto, GPIO18 Power.");

  Wire.begin(IIC_SDA, IIC_SCL);
  digitalWrite(TP_RESET, LOW);
  delay(30);
  digitalWrite(TP_RESET, HIGH);
  delay(50);
  delay(1000);
  Wire.begin(IIC_SDA, IIC_SCL);
  touch.setPins(TP_RESET, TP_INT);
  if (!touch.begin(Wire, 0x5A, IIC_SDA, IIC_SCL)) {
    Serial.println("Touch controller is not online.");
    while (true) delay(1000);
  }
  Serial.printf("Touch model: %s\n", touch.getModelName());
  // Keep the known-good touch reset/wake behavior used in the working baseline.
  touch.setMaxCoordinates(466, 466);
  touch.setMirrorXY(true, true);
  attachInterrupt(TP_INT, []() { touchPending = true; }, FALLING);

  gfx->begin();
  gfx->setBrightness(200);
  screenWidth = gfx->width();
  screenHeight = gfx->height();
  lv_init();

  loadUiSettings();
  initWeightFont();
  const uint32_t drawBufferAllocationPixels = screenWidth * (LVGL_DRAW_BUF_LINES + LVGL_DRAW_BUF_EXTRA_LINES);
  rotatedFlushBufferPixels = drawBufferAllocationPixels;
  rotatedFlushBuffer = (uint16_t *)heap_caps_malloc(
      rotatedFlushBufferPixels * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rotatedFlushBuffer) {
    Serial.printf("Could not allocate display rotation buffer (%u bytes in PSRAM).\n",
                  (unsigned)(rotatedFlushBufferPixels * sizeof(uint16_t)));
    while (true) delay(1000);
  }
  lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(drawBufferAllocationPixels * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(drawBufferAllocationPixels * sizeof(lv_color_t), MALLOC_CAP_DMA);
  if (!buf1 || !buf2) {
    Serial.printf("Could not allocate LVGL draw buffers (%lu bytes each, DMA heap free=%u).\n",
                  (unsigned long)(drawBufferAllocationPixels * sizeof(lv_color_t)),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));
    while (true) delay(1000);
  }
  // Let LVGL use the full DMA allocation, including the rotation headroom.
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, drawBufferAllocationPixels);

  static lv_disp_drv_t displayDriver;
  lv_disp_drv_init(&displayDriver);
  displayDriver.hor_res = screenWidth;
  displayDriver.ver_res = screenHeight;
  displayDriver.flush_cb = displayFlush;
  displayDriver.rounder_cb = displayRounder;
  displayDriver.draw_buf = &draw_buf;
  // Leave LVGL rotation disabled. The partial-tile transpose in displayFlush()
  // rotates the panel output without LVGL's glitch-prone software rotation path.
  displayDriver.sw_rotate = 0;
  lv_disp_drv_register(&displayDriver);

  static lv_indev_drv_t inputDriver;
  lv_indev_drv_init(&inputDriver);
  inputDriver.type = LV_INDEV_TYPE_POINTER;
  inputDriver.read_cb = touchRead;
  lv_indev_drv_register(&inputDriver);

  const esp_timer_create_args_t tickArgs = {.callback = &lvglTick, .name = "lvgl_tick"};
  esp_timer_handle_t tickTimer = nullptr;
  esp_timer_create(&tickArgs, &tickTimer);
  esp_timer_start_periodic(tickTimer, LVGL_TICK_MS * 1000);

  createUi();
  appQueue = xQueueCreate(32, sizeof(BrewEvent));
  blePacketQueue = xQueueCreate(16, sizeof(ScalePacket));
  commandQueue = xQueueCreate(8, sizeof(BrewCommand));
  if (!appQueue || !blePacketQueue || !commandQueue) {
    Serial.println("Could not allocate BLE event queues.");
    while (true) delay(1000);
  }
  xTaskCreatePinnedToCore(bleTask, "dot_ble", 8192, nullptr, 1, nullptr, 0);
  Serial.println("Setup complete; waiting for DOT.");
}

void loop() {
  processAppEvents();
  updateBrewSafety();
  static uint32_t lastInfoRefresh = 0;
  if (infoPageActive && (uint32_t)(millis() - lastInfoRefresh) >= 1000) {
    lastInfoRefresh = millis();
    updateInfoPage();
  }
  lv_timer_handler();
  updateCombinedButtonHold();
  updateModeButtonHold();
  delay(5);
}

// =============================================================================
// ESP32-S3 WEATHER ADVICE STATION
//
// Reads temperature, humidity, air pressure, UV index and light level from
// three I2C sensors and turns them into ONE plain-English piece of advice
// ("Rain is coming - grab an umbrella!") shown as a big icon + sentence on a
// TFT screen. A second screen shows the raw numbers for anyone who wants them.
//
// Two buttons swap between the two screens. Bad or missing sensor readings
// are detected and the display falls back to the last good reading instead
// of showing garbage values.
// =============================================================================

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>

// ---------- Pin constants ----------
const int      BUTTON_FRIENDLY_PIN = 1;   // D1 - shows the friendly advice screen
const int      BUTTON_RAWDATA_PIN  = 2;   // D2 - shows the raw sensor data screen
const int      TFT_BACKLIGHT_PIN   = TFT_BACKLITE;
const uint16_t SCREEN_WIDTH        = 240;
const uint16_t SCREEN_HEIGHT       = 135;

// ---------- Sensor / calibration constants ----------
const float    PASCALS_PER_HPA     = 100.0F;  // BMP280 returns Pa; divide to get hPa
const float    UV_SENSITIVITY      = 2300.0F; // LTR390 datasheet conversion factor
const float    ALS_GAIN_FACTOR     = 3.0F;
const float    ALS_INT_FACTOR      = 1.0F;
const float    LUX_WFA_COEFF       = 0.6F;
const unsigned long SENSOR_SWITCH_DELAY_MS = 120; // settle time after changing LTR390 mode
const unsigned long LOOP_DELAY_MS          = 2000;
const unsigned long DEBOUNCE_MS            = 250;

// Pressure-trend thresholds (hPa change over the tracking window)
const float    PRESSURE_FALLING_FAST = -2.0F;  // sharp drop - rain likely soon
const float    PRESSURE_FALLING_SLOW = -0.7F;  // gentle drop - rain possible later
const float    PRESSURE_RISING_SLOW  = 0.7F;   // gentle rise - improving
const int      PRESSURE_HISTORY_SIZE = 5;      // ~10 sec of readings at 2s loop

// ---------- Valid sensor ranges (used to catch disconnected/faulty sensors) ----------
// Anything outside these physical ranges, or a NaN reading, is treated as an
// invalid sample so a glitch never gets shown to the user as real weather data.
const float    TEMP_MIN_C        = -40.0F, TEMP_MAX_C        = 85.0F;
const float    HUMIDITY_MIN_PCT  = 0.0F,   HUMIDITY_MAX_PCT  = 100.0F;
const float    PRESSURE_MIN_HPA  = 300.0F, PRESSURE_MAX_HPA  = 1100.0F;
const float    UV_INDEX_MIN      = 0.0F,   UV_INDEX_MAX      = 20.0F;
const float    LIGHT_LUX_MIN     = 0.0F,   LIGHT_LUX_MAX     = 100000.0F;

// ---------- Advice trigger thresholds ----------
// Named the same way as the validity ranges above, rather than left as bare
// numbers inside buildAdviceMessage()'s if-statements.
const float    ADVICE_UV_DANGEROUS        = 8.0F;   // uvIndex at/above this - "wear sunscreen now"
const float    ADVICE_UV_STRONG           = 6.0F;   // uvIndex at/above this - "sunscreen recommended"
const float    ADVICE_HOT_HUMID_TEMP_C    = 25.0F;  // paired with humidity below for muggy-day advice
const float    ADVICE_HOT_HUMID_HUMIDITY_PCT = 50.0F;
const float    ADVICE_HOT_TEMP_C          = 27.0F;  // "hot day" advice on its own
const float    ADVICE_COLD_TEMP_C         = 15.0F;  // below this - "wear a jacket"
const float    ADVICE_DARK_LUX            = 2.0F;   // below this - "turn on the lights"

// ---------- Text size constants ----------
const int      HEADER_TEXT_SIZE   = 2;
const int      SENTENCE_MAX_SIZE  = 2;   // largest size tried for the advice sentence
const int      FOOTER_TEXT_SIZE   = 1;
const int      CHAR_PIXEL_WIDTH   = 6;   // base width of one character at text size 1
const int      MAX_TEXT_LINES     = 6;   // real capacity of the lines[] buffer in drawWrappedCentredText

// ---------- Colours ----------
const uint16_t COLOR_BG       = ST77XX_BLACK;
const uint16_t COLOR_HEADER   = ST77XX_BLUE;
const uint16_t COLOR_TEXT     = ST77XX_WHITE;
const uint16_t COLOR_GOOD     = ST77XX_GREEN;
const uint16_t COLOR_WARN     = ST77XX_YELLOW;
const uint16_t COLOR_BAD      = ST77XX_RED;
const uint16_t COLOR_LABEL    = ST77XX_CYAN;

// ---------- Sensor objects ----------
Adafruit_AHTX0  aht;
Adafruit_BMP280 bmp;
Adafruit_LTR390 ltr;
Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);

// ---------- Data structure ----------
// Groups one full set of sensor readings together so they can be passed
// around and returned from functions as a single unit instead of five
// separate loose variables.
struct SensorReading {
  float temperatureC;
  float humidityPct;
  float pressureHPa;
  float uvIndex;
  float lightLux;
};

// Icons available to represent advice at a glance, drawn with basic shapes
enum AdviceIcon { ICON_RAIN, ICON_SUN, ICON_CLOUD, ICON_BULB, ICON_SNOWFLAKE };

// One complete piece of advice: what to say, how urgent/coloured it is, and
// which icon best represents it.
struct AdviceMessage {
  String     sentence;
  uint16_t   color;
  AdviceIcon icon;
};

enum ScreenMode { SCREEN_FRIENDLY, SCREEN_RAWDATA };
ScreenMode currentScreen = SCREEN_FRIENDLY;

int  lastFriendlyState = LOW;
int  lastRawDataState  = LOW;
unsigned long lastFriendlyPress = 0;
unsigned long lastRawDataPress  = 0;

// ---------- Pressure history for trend / weather-advice detection ----------
float pressureHistory[PRESSURE_HISTORY_SIZE];
int   pressureHistoryIndex = 0;
bool  pressureHistoryFull  = false;

// ---------- Last-known-good reading, used as a fallback when a sample fails validation ----------
SensorReading lastGoodReading = { 20.0F, 50.0F, 1013.0F, 0.0F, 100.0F }; // sensible defaults until first good read
bool          haveGoodReading = false;

// =============================================================================
// isReadingValid - checks a sensor reading against physically-possible ranges
// and rejects NaN values (which the Adafruit libraries return when a sensor
// read fails). This is what stops a loose wire or a glitchy sample from being
// shown to the user as real weather data.
// =============================================================================
bool isReadingValid(const SensorReading &data) {
  if (isnan(data.temperatureC) || isnan(data.humidityPct) || isnan(data.pressureHPa) ||
      isnan(data.uvIndex) || isnan(data.lightLux)) {
    return false;
  }

  if (data.temperatureC < TEMP_MIN_C || data.temperatureC > TEMP_MAX_C)       return false;
  if (data.humidityPct  < HUMIDITY_MIN_PCT || data.humidityPct > HUMIDITY_MAX_PCT) return false;
  if (data.pressureHPa  < PRESSURE_MIN_HPA || data.pressureHPa > PRESSURE_MAX_HPA) return false;
  if (data.uvIndex      < UV_INDEX_MIN || data.uvIndex > UV_INDEX_MAX)        return false;
  if (data.lightLux     < LIGHT_LUX_MIN || data.lightLux > LIGHT_LUX_MAX)     return false;

  return true;
}

// =============================================================================
// readAllSensors - takes one reading from each of the three sensors (AHT20
// temperature/humidity, BMP280 pressure, LTR390 UV/light) and packages the
// results into a single SensorReading to hand back to the caller.
// =============================================================================
SensorReading readAllSensors() {
  SensorReading data;

  sensors_event_t humidityEvent, tempEvent;
  aht.getEvent(&humidityEvent, &tempEvent);
  data.temperatureC = tempEvent.temperature;
  data.humidityPct  = humidityEvent.relative_humidity;

  data.pressureHPa = bmp.readPressure() / PASCALS_PER_HPA;

  // Default to 0 in case the LTR390 has no new data ready this cycle
  data.uvIndex  = 0;
  data.lightLux = 0;

  ltr.setMode(LTR390_MODE_UVS);
  delay(SENSOR_SWITCH_DELAY_MS);
  if (ltr.newDataAvailable()) {
    uint32_t rawUV = ltr.readUVS();
    data.uvIndex = (float)rawUV / UV_SENSITIVITY;
  }

  ltr.setMode(LTR390_MODE_ALS);
  delay(SENSOR_SWITCH_DELAY_MS);
  if (ltr.newDataAvailable()) {
    uint32_t rawALS = ltr.readALS();
    data.lightLux = (LUX_WFA_COEFF * (float)rawALS) / (ALS_GAIN_FACTOR * ALS_INT_FACTOR);
  }

  return data;
}

// =============================================================================
// getCurrentReading - the single point where the rest of the program gets its
// sensor data from. It takes a fresh reading, validates it, and either
// accepts it as the new "last known good" reading or falls back to the
// previous good one. Returns the reading to use and reports (via the
// isStale output parameter) whether that reading is a fallback.
// =============================================================================
SensorReading getCurrentReading(bool &isStale) {
  SensorReading freshReading = readAllSensors();

  if (isReadingValid(freshReading)) {
    lastGoodReading = freshReading;
    haveGoodReading = true;
    isStale = false;
    return freshReading;
  }

  // Invalid sample: warn on Serial and fall back to the last good reading
  // (or the startup default if a good reading has never been captured yet).
  Serial.println(F("[WARN] Invalid sensor reading rejected - using last known good data"));
  isStale = true;
  return lastGoodReading;
}

// =============================================================================
// updatePressureTrend - tracks pressure over time in a circular buffer and
// returns the change in hPa between the oldest and newest stored readings.
// A positive result means pressure is rising, negative means it's falling.
// =============================================================================
float updatePressureTrend(float currentPressure) {
  pressureHistory[pressureHistoryIndex] = currentPressure;
  pressureHistoryIndex = (pressureHistoryIndex + 1) % PRESSURE_HISTORY_SIZE;
  if (pressureHistoryIndex == 0) pressureHistoryFull = true;

  if (!pressureHistoryFull) return 0; // not enough data collected yet

  int oldestIndex = pressureHistoryIndex;
  float oldestPressure = pressureHistory[oldestIndex];
  return currentPressure - oldestPressure;
}

// =============================================================================
// buildAdviceMessage - takes ALL the sensor data and the pressure trend and
// returns ONE clear sentence telling the user what to do, picked by priority
// (most important / urgent advice wins). This is the only thing the friendly
// screen needs to know how to draw.
// =============================================================================
AdviceMessage buildAdviceMessage(const SensorReading &data, float pressureTrend) {
  // Priority 1: rain warning - most actionable, affects whether you go outside
  if (pressureTrend <= PRESSURE_FALLING_FAST) {
    return { "Rain is coming - grab an umbrella!", COLOR_BAD, ICON_RAIN };
  }
  if (pressureTrend <= PRESSURE_FALLING_SLOW) {
    return { "Showers possible - keep an umbrella handy", COLOR_WARN, ICON_RAIN };
  }

  // Priority 2: dangerous UV - safety advice
  if (data.uvIndex >= ADVICE_UV_DANGEROUS) {
    return { "UV is very high - wear sunscreen now!", COLOR_BAD, ICON_SUN };
  }
  if (data.uvIndex >= ADVICE_UV_STRONG) {
    return { "Strong sun out - sunscreen recommended", COLOR_WARN, ICON_SUN };
  }

  // Priority 3: temperature comfort
  if (data.temperatureC > ADVICE_HOT_HUMID_TEMP_C && data.humidityPct > ADVICE_HOT_HUMID_HUMIDITY_PCT) {
    return { "Hot and humid - drink plenty of water", COLOR_WARN, ICON_SUN };
  }
  if (data.temperatureC > ADVICE_HOT_TEMP_C) {
    return { "It's a hot day - dress light and stay cool", COLOR_WARN, ICON_SUN };
  }
  if (data.temperatureC < ADVICE_COLD_TEMP_C) {
    return { "Feeling cold - wear a warm jacket", COLOR_LABEL, ICON_SNOWFLAKE };
  }

  // Priority 4: lighting
  if (data.lightLux < ADVICE_DARK_LUX) {
    return { "It's dark - turn on the lights", COLOR_WARN, ICON_BULB };
  }

  // Priority 5: pressure rising - good news, nothing urgent needed
  if (pressureTrend >= PRESSURE_RISING_SLOW) {
    return { "Skies clearing - a great time to be outside!", COLOR_GOOD, ICON_SUN };
  }

  // Default - everything is comfortable, no action needed
  return { "Conditions look comfortable - enjoy your day!", COLOR_GOOD, ICON_CLOUD };
}

// =============================================================================
// printDataToSerial - Serial Monitor raw data table, always printed
// regardless of which screen is showing. Also flags when the values being
// shown are a fallback (stale) reading rather than a fresh sample.
// =============================================================================
void printDataToSerial(const SensorReading &data, float pressureTrend, bool isStale) {
  Serial.println();
  Serial.println(F("=========== SENSOR READINGS ==========="));
  if (isStale) {
    Serial.println(F("[STALE - showing last known good reading]"));
  }
  Serial.printf("%-12s %8.1f %s\n", "Temperature", data.temperatureC, "C");
  Serial.printf("%-12s %8.1f %s\n", "Humidity",    data.humidityPct,  "%");
  Serial.printf("%-12s %8.1f %s\n", "Pressure",    data.pressureHPa,  "hPa");
  Serial.printf("%-12s %8.2f %s\n", "Pres. Trend", pressureTrend,     "hPa");
  Serial.printf("%-12s %8.2f %s\n", "UV Index",    data.uvIndex,      "");
  Serial.printf("%-12s %8.1f %s\n", "Light",       data.lightLux,     "lux");
  Serial.println(F("========================================"));
}

// =============================================================================
// drawHeader - draws the coloured title bar shown at the top of both screens
// =============================================================================
void drawHeader(const char* title) {
  tft.fillRect(0, 0, SCREEN_WIDTH, 22, COLOR_HEADER);
  tft.setCursor(4, 3);
  tft.setTextSize(HEADER_TEXT_SIZE);
  tft.setTextColor(COLOR_TEXT);
  tft.print(title);
}

// =============================================================================
// Hand-drawn icons using basic GFX shapes (no emojis avaliable on TFT).
// Each is drawn centred at (cx, cy) in the given colour.
// =============================================================================
void drawSunIcon(int cx, int cy, uint16_t color) {
  const int radius = 9;
  tft.fillCircle(cx, cy, radius, color);
  for (int i = 0; i < 8; i++) {
    float angle = i * (PI / 4.0);
    int x1 = cx + (int)((radius + 3) * cos(angle));
    int y1 = cy + (int)((radius + 3) * sin(angle));
    int x2 = cx + (int)((radius + 9) * cos(angle));
    int y2 = cy + (int)((radius + 9) * sin(angle));
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

void drawCloudIcon(int cx, int cy, uint16_t color) {
  tft.fillCircle(cx - 8, cy + 2, 8, color);
  tft.fillCircle(cx + 4, cy - 4, 10, color);
  tft.fillCircle(cx + 14, cy + 2, 7, color);
  tft.fillRect(cx - 15, cy + 2, 38, 9, color);
}

void drawUmbrellaIcon(int cx, int cy, uint16_t color) {
  const int radius = 12;

  // Canopy - top half of a circle
  tft.fillCircle(cx, cy, radius, color);
  tft.fillRect(cx - radius, cy, radius * 2, radius, COLOR_BG);

  // Scalloped bottom edge
  for (int i = -1; i <= 1; i++) {
    tft.fillCircle(cx + (i * (radius / 1.5)), cy, 2, COLOR_BG);
  }

  // Pole
  tft.drawLine(cx, cy, cx, cy + 16, color);

  // Handle hook
  tft.drawLine(cx, cy + 16, cx - 5, cy + 16, color);
  tft.drawLine(cx - 5, cy + 16, cx - 5, cy + 11, color);

  // Raindrops beside the umbrella
  tft.drawLine(cx + 19, cy - 4, cx + 16, cy + 3, color);
  tft.drawLine(cx + 26, cy - 2, cx + 23, cy + 5, color);
}

void drawBulbIcon(int cx, int cy, uint16_t color) {
  tft.fillCircle(cx, cy - 2, 10, color);
  tft.fillRect(cx - 5, cy + 6, 10, 6, color);
  tft.drawLine(cx - 5, cy + 13, cx + 5, cy + 13, color);
  for (int i = 0; i < 6; i++) {
    float angle = i * (PI / 3.0);
    int x1 = cx + (int)(13 * cos(angle));
    int y1 = (cy - 2) + (int)(13 * sin(angle));
    int x2 = cx + (int)(18 * cos(angle));
    int y2 = (cy - 2) + (int)(18 * sin(angle));
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

void drawSnowflakeIcon(int cx, int cy, uint16_t color) {
  const int armLength = 13;
  for (int i = 0; i < 3; i++) {
    float angle = i * (PI / 3.0);
    int x1 = cx - (int)(armLength * cos(angle));
    int y1 = cy - (int)(armLength * sin(angle));
    int x2 = cx + (int)(armLength * cos(angle));
    int y2 = cy + (int)(armLength * sin(angle));
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

// Picks the right hand-drawn icon function based on the AdviceIcon enum value
void drawAdviceIcon(AdviceIcon icon, int cx, int cy, uint16_t color) {
  switch (icon) {
    case ICON_RAIN:       drawUmbrellaIcon(cx, cy, color);     break;
    case ICON_SUN:         drawSunIcon(cx, cy, color);           break;
    case ICON_CLOUD:       drawCloudIcon(cx, cy, ST77XX_WHITE);  break;
    case ICON_BULB:        drawBulbIcon(cx, cy, color);          break;
    case ICON_SNOWFLAKE:   drawSnowflakeIcon(cx, cy, color);     break;
  }
}

// =============================================================================
// drawWrappedCentredText - wraps a sentence onto multiple lines on word
// boundaries, centres each line horizontally, and lays the block out below
// the icon. Keeps everything big and easy to read regardless of sentence
// length.
//
// maxLines is clamped to MAX_TEXT_LINES (the real capacity of the lines[]
// buffer below) before it's used anywhere. Without this, a caller-supplied
// maxLines bigger than the array size - e.g. drawFriendlyScreen()'s layout
// maths returning 5 while lines[] only held 4 - would let the wrapping loop
// write one element past the end of the array. Clamping here means that can
// never happen, no matter what value a caller passes in.
// =============================================================================
void drawWrappedCentredText(String message, int topY, uint16_t color, int textSize, int maxLines) {
  if (maxLines > MAX_TEXT_LINES) {
    maxLines = MAX_TEXT_LINES;
  }

  int charWidth     = CHAR_PIXEL_WIDTH * textSize;
  int charsPerLine  = (SCREEN_WIDTH - 16) / charWidth;

  String lines[MAX_TEXT_LINES];
  int lineCount = 0;
  String remaining = message;

  while (remaining.length() > 0 && lineCount < maxLines) {
    if ((int)remaining.length() <= charsPerLine) {
      lines[lineCount++] = remaining;
      break;
    }
    int breakPoint = remaining.substring(0, charsPerLine).lastIndexOf(' ');
    if (breakPoint <= 0) breakPoint = charsPerLine;
    lines[lineCount++] = remaining.substring(0, breakPoint);
    remaining = remaining.substring(breakPoint + 1);
  }

  int lineHeight = (8 * textSize) + 3;

  tft.setTextSize(textSize);
  tft.setTextColor(color);

  for (int i = 0; i < lineCount; i++) {
    int textWidth = lines[i].length() * charWidth;
    int startX = (SCREEN_WIDTH - textWidth) / 2;
    if (startX < 4) startX = 4;
    tft.setCursor(startX, topY + (i * lineHeight));
    tft.println(lines[i]);
  }
}

// =============================================================================
// drawFriendlyScreen - Screen 1: ONE big icon + ONE clear sentence telling
// the user what to do, generated from all the sensor data. If the current
// reading is a stale fallback, the footer hint says so instead of the
// normal "press for raw data" prompt.
// =============================================================================
void drawFriendlyScreen(const SensorReading &data, float pressureTrend, bool isStale) {
  tft.fillScreen(COLOR_BG);
  drawHeader("TODAY'S ADVICE");

  AdviceMessage advice = buildAdviceMessage(data, pressureTrend);

  // Icon centred near the top, below the header (kept compact to leave
  // more vertical room for the sentence below)
  drawAdviceIcon(advice.icon, SCREEN_WIDTH / 2, 38, advice.color);

  // Sentence wrapped and centred below the icon, sized to fit ABOVE the
  // footer hint with no overlap. Text area is between sentenceTopY and
  // footerTopY, so we pick the largest size and shrink lines to fit inside.
  const int sentenceTopY = 60;
  const int footerTopY   = SCREEN_HEIGHT - 16; // leave room for footer text
  const int availableHeight = footerTopY - sentenceTopY;

  int textSize = SENTENCE_MAX_SIZE;
  int lineHeight = (8 * textSize) + 3;
  int maxLines = availableHeight / lineHeight;
  if (maxLines < 1) maxLines = 1;

  int charWidth    = CHAR_PIXEL_WIDTH * textSize;
  int charsPerLine = (SCREEN_WIDTH - 16) / charWidth;
  int estimatedLines = (advice.sentence.length() / charsPerLine) + 1;

  // If the sentence won't fit at this size within the allowed lines, shrink
  if (estimatedLines > maxLines) {
    textSize = 1;
    lineHeight = (8 * textSize) + 3;
    maxLines = availableHeight / lineHeight;
    if (maxLines < 1) maxLines = 1;
  }

  drawWrappedCentredText(advice.sentence, sentenceTopY, advice.color, textSize, maxLines);

  // Footer hint - always visible, centred, with a clear gap above it
  tft.setTextSize(FOOTER_TEXT_SIZE);
  tft.setTextColor(isStale ? COLOR_BAD : COLOR_WARN);
  const char* hint = isStale ? "Sensor error - showing last reading" : "Press D2 for raw data";
  int hintWidth = strlen(hint) * CHAR_PIXEL_WIDTH * FOOTER_TEXT_SIZE;
  tft.setCursor((SCREEN_WIDTH - hintWidth) / 2, SCREEN_HEIGHT - 11);
  tft.print(hint);
}

// =============================================================================
// drawRawDataScreen - Screen 2: raw numeric sensor data, one row per value.
// Rows are drawn with a small local helper (a lambda) so the label/value
// layout logic isn't repeated five times.
// =============================================================================
void drawRawDataScreen(const SensorReading &data, float pressureTrend, bool isStale) {
  tft.fillScreen(COLOR_BG);
  drawHeader("RAW DATA");

  int rowY = 26;
  const int rowHeight = 16;

  tft.setTextWrap(false); // prevent long values wrapping into the row below

  auto drawRow = [&](const char* label, String value) {
    tft.setCursor(4, rowY);
    tft.setTextSize(2);
    tft.setTextColor(COLOR_LABEL);
    tft.print(label);
    tft.setCursor(140, rowY);
    tft.setTextColor(isStale ? COLOR_BAD : COLOR_GOOD);
    tft.println(value);
    rowY += rowHeight;
  };

  drawRow("Temp:",  String(data.temperatureC, 1) + "C");
  drawRow("Hum:",   String(data.humidityPct, 1) + "%");
  drawRow("Pres:",  String(data.pressureHPa, 0));
  drawRow("UV:",    String(data.uvIndex, 1));
  drawRow("Light:", String(data.lightLux, 0));

  tft.setCursor(4, SCREEN_HEIGHT - 10);
  tft.setTextSize(FOOTER_TEXT_SIZE);
  tft.setTextColor(COLOR_WARN);
  tft.print(isStale ? "(stale) <- D1: summary" : "<- D1: summary");
}

// Chooses which screen to draw based on the currentScreen state, so the
// main loop doesn't need to know about either screen's drawing details.
void updateDisplay(const SensorReading &data, float pressureTrend, bool isStale) {
  if (currentScreen == SCREEN_FRIENDLY) {
    drawFriendlyScreen(data, pressureTrend, isStale);
  } else {
    drawRawDataScreen(data, pressureTrend, isStale);
  }
}

// =============================================================================
// checkButtons - reads both buttons with simple debouncing and switches
// currentScreen when a fresh press is detected. Returns true if the screen
// mode changed this call (available for callers that only want to redraw on
// a genuine change).
// =============================================================================
bool checkButtons() {
  bool screenChanged = false;
  unsigned long now = millis();

  int friendlyState = digitalRead(BUTTON_FRIENDLY_PIN);
  int rawDataState  = digitalRead(BUTTON_RAWDATA_PIN);

  if (friendlyState == HIGH && lastFriendlyState == LOW && (now - lastFriendlyPress) > DEBOUNCE_MS) {
    currentScreen = SCREEN_FRIENDLY;
    lastFriendlyPress = now;
    screenChanged = true;
  }

  if (rawDataState == HIGH && lastRawDataState == LOW && (now - lastRawDataPress) > DEBOUNCE_MS) {
    currentScreen = SCREEN_RAWDATA;
    lastRawDataPress = now;
    screenChanged = true;
  }

  lastFriendlyState = friendlyState;
  lastRawDataState  = rawDataState;

  return screenChanged;
}

// =============================================================================
// setup - runs once at power-on. Starts Serial and I2C, configures the
// buttons and backlight, initialises the TFT, and brings up all three
// sensors. Each sensor halts the program with a clear error message if it
// fails to initialise, since the rest of the program cannot run without it.
// =============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin();

  pinMode(BUTTON_FRIENDLY_PIN, INPUT_PULLDOWN);
  pinMode(BUTTON_RAWDATA_PIN, INPUT_PULLDOWN);

  pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN, HIGH);
  tft.init(SCREEN_HEIGHT, SCREEN_WIDTH);
  tft.setRotation(3);
  tft.fillScreen(COLOR_BG);

  Serial.println(F("ESP32-S3 Feather - Sensor Init"));
  Serial.println(F("================================"));

  if (!aht.begin()) {
    Serial.println(F("[ERROR] AHT20 not found! Check wiring."));
    while (1) delay(100);
  }
  Serial.println(F("[OK] AHT20 ready"));

  if (!bmp.begin(0x76)) {
    if (!bmp.begin(0x77)) {
      Serial.println(F("[ERROR] BMP280 not found! Check wiring / address."));
      while (1) delay(100);
    }
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  Serial.println(F("[OK] BMP280 ready"));

  if (!ltr.begin()) {
    Serial.println(F("[ERROR] LTR390 not found! Check wiring."));
    while (1) delay(100);
  }
  ltr.setMode(LTR390_MODE_UVS);
  ltr.setGain(LTR390_GAIN_3);
  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  Serial.println(F("[OK] LTR390 ready"));

  Serial.println();
  Serial.println(F("Press D1 = friendly screen, D2 = raw data screen"));
  Serial.println(F("----------------------------------------------------------"));
}

// =============================================================================
// loop - runs continuously. Checks the buttons, takes a validated sensor
// reading (falling back to the last good one if the fresh sample fails
// validation), updates the pressure trend, and refreshes both the Serial
// Monitor and the TFT display.
// =============================================================================
void loop() {
  checkButtons();

  bool isStale = false;
  SensorReading currentReading = getCurrentReading(isStale);
  float pressureTrend = updatePressureTrend(currentReading.pressureHPa);

  printDataToSerial(currentReading, pressureTrend, isStale);
  updateDisplay(currentReading, pressureTrend, isStale);

  delay(LOOP_DELAY_MS);
}
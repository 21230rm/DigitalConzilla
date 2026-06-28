#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LTR390.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>

// ---------- Pin constants ----------
const int      BUTTON_FRIENDLY_PIN = 1;   // D1
const int      BUTTON_RAWDATA_PIN  = 2;   // D2
const int      TFT_BACKLIGHT_PIN   = TFT_BACKLITE;
const uint16_t SCREEN_WIDTH        = 240;
const uint16_t SCREEN_HEIGHT       = 135;

// ---------- Sensor / calibration constants ----------
const float    SEALEVEL_HPA        = 100.0F;
const float    UV_SENSITIVITY      = 2300.0F;
const float    ALS_GAIN_FACTOR     = 3.0F;
const float    ALS_INT_FACTOR      = 1.0F;
const float    LUX_WFA_COEFF       = 0.6F;
const unsigned long SENSOR_DELAY_MS = 120;
const unsigned long LOOP_DELAY_MS   = 2000;
const unsigned long DEBOUNCE_MS     = 250;

// Pressure-trend thresholds (hPa change over the tracking window)
const float    PRESSURE_FALLING_FAST = -2.0F;  // sharp drop - rain likely soon
const float    PRESSURE_FALLING_SLOW = -0.7F;  // gentle drop - rain possible later
const float    PRESSURE_RISING_SLOW  = 0.7F;   // gentle rise - improving
const int      PRESSURE_HISTORY_SIZE = 5;       // ~10 sec of readings at 2s loop

// ---------- Text size constants ----------
const int      HEADER_TEXT_SIZE   = 2;
const int      SENTENCE_MAX_SIZE  = 2;   // largest size tried for the advice sentence
const int      FOOTER_TEXT_SIZE   = 1;
const int      CHAR_PIXEL_WIDTH   = 6;   // base width of one character at text size 1

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

// ---------------------------------------------------------------------------
SensorReading readAllSensors() {
  SensorReading data;

  sensors_event_t humidityEvent, tempEvent;
  aht.getEvent(&humidityEvent, &tempEvent);
  data.temperatureC = tempEvent.temperature;
  data.humidityPct  = humidityEvent.relative_humidity;

  data.pressureHPa = bmp.readPressure() / SEALEVEL_HPA;

  data.uvIndex  = 0;
  data.lightLux = 0;

  ltr.setMode(LTR390_MODE_UVS);
  delay(SENSOR_DELAY_MS);
  if (ltr.newDataAvailable()) {
    uint32_t rawUV = ltr.readUVS();
    data.uvIndex = (float)rawUV / UV_SENSITIVITY;
  }

  ltr.setMode(LTR390_MODE_ALS);
  delay(SENSOR_DELAY_MS);
  if (ltr.newDataAvailable()) {
    uint32_t rawALS = ltr.readALS();
    data.lightLux = (LUX_WFA_COEFF * (float)rawALS) / (ALS_GAIN_FACTOR * ALS_INT_FACTOR);
  }

  return data;
}

// ---------------------------------------------------------------------------
// updatePressureTrend - tracks pressure over time in a circular buffer and
// returns the change in hPa between the oldest and newest stored readings.
// ---------------------------------------------------------------------------
float updatePressureTrend(float currentPressure) {
  pressureHistory[pressureHistoryIndex] = currentPressure;
  pressureHistoryIndex = (pressureHistoryIndex + 1) % PRESSURE_HISTORY_SIZE;
  if (pressureHistoryIndex == 0) pressureHistoryFull = true;

  if (!pressureHistoryFull) return 0; // not enough data collected yet

  int oldestIndex = pressureHistoryIndex;
  float oldestPressure = pressureHistory[oldestIndex];
  return currentPressure - oldestPressure;
}

// ---------------------------------------------------------------------------
// buildAdviceMessage - takes ALL the sensor data and the pressure trend and
// returns ONE clear sentence telling the user what to do, picked by priority
// (most important / urgent advice wins). This is the only thing the friendly
// screen needs to know how to draw.
// ---------------------------------------------------------------------------
AdviceMessage buildAdviceMessage(const SensorReading &data, float pressureTrend) {
  // Priority 1: rain warning - most actionable, affects whether you go outside
  if (pressureTrend <= PRESSURE_FALLING_FAST) {
    return { "Rain is coming - grab an umbrella!", COLOR_BAD, ICON_RAIN };
  }
  if (pressureTrend <= PRESSURE_FALLING_SLOW) {
    return { "Showers possible - keep an umbrella handy", COLOR_WARN, ICON_RAIN };
  }

  // Priority 2: dangerous UV - safety advice
  if (data.uvIndex >= 8) {
    return { "UV is very high - wear sunscreen now!", COLOR_BAD, ICON_SUN };
  }
  if (data.uvIndex >= 6) {
    return { "Strong sun out - sunscreen recommended", COLOR_WARN, ICON_SUN };
  }

  // Priority 3: temperature comfort
  if (data.temperatureC > 28 && data.humidityPct > 60) {
    return { "Hot and humid - drink plenty of water", COLOR_WARN, ICON_SUN };
  }
  if (data.temperatureC > 28) {
    return { "It's a hot day - dress light and stay cool", COLOR_WARN, ICON_SUN };
  }
  if (data.temperatureC < 15) {
    return { "Feeling cold - wear a warm jacket", COLOR_LABEL, ICON_SNOWFLAKE };
  }

  // Priority 4: lighting
  if (data.lightLux < 10) {
    return { "It's dark - turn on the lights", COLOR_WARN, ICON_BULB };
  }

  // Priority 5: pressure rising - good news, nothing urgent needed
  if (pressureTrend >= PRESSURE_RISING_SLOW) {
    return { "Skies clearing - a great time to be outside!", COLOR_GOOD, ICON_SUN };
  }

  // Default - everything is comfortable, no action needed
  return { "Conditions look comfortable - enjoy your day!", COLOR_GOOD, ICON_CLOUD };
}

// ---------------------------------------------------------------------------
// Serial Monitor - raw data table, always printed regardless of screen mode
// ---------------------------------------------------------------------------
void printDataToSerial(const SensorReading &data, float pressureTrend) {
  Serial.println();
  Serial.println(F("=========== SENSOR READINGS ==========="));
  Serial.printf("%-12s %8.1f %s\n", "Temperature", data.temperatureC, "C");
  Serial.printf("%-12s %8.1f %s\n", "Humidity",    data.humidityPct,  "%");
  Serial.printf("%-12s %8.1f %s\n", "Pressure",    data.pressureHPa,  "hPa");
  Serial.printf("%-12s %8.2f %s\n", "Pres. Trend", pressureTrend,     "hPa");
  Serial.printf("%-12s %8.2f %s\n", "UV Index",    data.uvIndex,      "");
  Serial.printf("%-12s %8.1f %s\n", "Light",       data.lightLux,     "lux");
  Serial.println(F("========================================"));
}

// ---------------------------------------------------------------------------
// Header bar drawer
// ---------------------------------------------------------------------------
void drawHeader(const char* title) {
  tft.fillRect(0, 0, SCREEN_WIDTH, 22, COLOR_HEADER);
  tft.setCursor(4, 3);
  tft.setTextSize(HEADER_TEXT_SIZE);
  tft.setTextColor(COLOR_TEXT);
  tft.print(title);
}

// ---------------------------------------------------------------------------
// Hand-drawn icons using basic GFX shapes (no emoji font available on TFT)
// Each is drawn centred at (cx, cy)
// ---------------------------------------------------------------------------
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

void drawAdviceIcon(AdviceIcon icon, int cx, int cy, uint16_t color) {
  switch (icon) {
    case ICON_RAIN:      drawUmbrellaIcon(cx, cy, color);  break;
    case ICON_SUN:        drawSunIcon(cx, cy, color);        break;
    case ICON_CLOUD:      drawCloudIcon(cx, cy, ST77XX_WHITE); break;
    case ICON_BULB:       drawBulbIcon(cx, cy, color);       break;
    case ICON_SNOWFLAKE:  drawSnowflakeIcon(cx, cy, color);  break;
  }
}

// ---------------------------------------------------------------------------
// drawWrappedCentredText - wraps a sentence onto multiple lines on word
// boundaries, centres each line horizontally, and centres the whole block
// vertically below the icon. Keeps everything big and easy to read.
// ---------------------------------------------------------------------------
void drawWrappedCentredText(String message, int topY, uint16_t color, int textSize, int maxLines) {
  int charWidth     = CHAR_PIXEL_WIDTH * textSize;
  int charsPerLine  = (SCREEN_WIDTH - 16) / charWidth;

  String lines[4];
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

// ---------------------------------------------------------------------------
// TFT Screen 1 - ONE big icon + ONE clear sentence telling the user what to
// do, generated from all the sensor data. This is the whole friendly screen.
// ---------------------------------------------------------------------------
void drawFriendlyScreen(const SensorReading &data, float pressureTrend) {
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
  tft.setTextColor(COLOR_WARN);
  const char* hint = "Press D2 for raw data";
  int hintWidth = strlen(hint) * CHAR_PIXEL_WIDTH * FOOTER_TEXT_SIZE;
  tft.setCursor((SCREEN_WIDTH - hintWidth) / 2, SCREEN_HEIGHT - 11);
  tft.print(hint);
}

// ---------------------------------------------------------------------------
// TFT Screen 2 - raw numeric data
// ---------------------------------------------------------------------------
void drawRawDataScreen(const SensorReading &data, float pressureTrend) {
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
    tft.setTextColor(COLOR_GOOD);
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
  tft.print("<- D1: summary");
}

void updateDisplay(const SensorReading &data, float pressureTrend) {
  if (currentScreen == SCREEN_FRIENDLY) {
    drawFriendlyScreen(data, pressureTrend);
  } else {
    drawRawDataScreen(data, pressureTrend);
  }
}

// ---------------------------------------------------------------------------
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

void loop() {
  checkButtons();

  SensorReading currentReading = readAllSensors();
  float pressureTrend = updatePressureTrend(currentReading.pressureHPa);

  printDataToSerial(currentReading, pressureTrend);
  updateDisplay(currentReading, pressureTrend);

  delay(LOOP_DELAY_MS);
}
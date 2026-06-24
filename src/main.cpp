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
const float    PRESSURE_RISING_FAST  = 2.0F;   // sharp rise - clearing up
const float    PRESSURE_RISING_SLOW  = 0.7F;   // gentle rise - improving
const int      PRESSURE_HISTORY_SIZE = 5;       // ~10 sec of readings at 2s loop

// ---------- Text size constants ----------
const int      HEADER_TEXT_SIZE   = 2;
const int      BIG_TEXT_SIZE       = 3;
const int      BODY_TEXT_SIZE     = 2;
const int      FOOTER_TEXT_SIZE   = 1;
const int      CHAR_PIXEL_WIDTH   = 6;

// ---------- Colours ----------
const uint16_t COLOR_BG       = ST77XX_BLACK;
const uint16_t COLOR_HEADER   = ST77XX_BLUE;
const uint16_t COLOR_TEXT     = ST77XX_WHITE;
const uint16_t COLOR_GOOD     = ST77XX_GREEN;
const uint16_t COLOR_WARN     = ST77XX_YELLOW;
const uint16_t COLOR_BAD      = ST77XX_RED;
const uint16_t COLOR_LABEL    = ST77XX_CYAN;
const uint16_t COLOR_SUN      = ST77XX_YELLOW;
const uint16_t COLOR_CLOUD    = ST77XX_WHITE;
const uint16_t COLOR_RAIN     = ST77XX_CYAN;

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

enum ScreenMode { SCREEN_FRIENDLY, SCREEN_RAWDATA };
ScreenMode currentScreen = SCREEN_FRIENDLY;

// Weather condition derived from pressure trend, used to pick an icon
enum WeatherIcon { ICON_RAIN, ICON_CLOUD, ICON_SUN };

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

String getWeatherAdvice(float pressureTrend) {
  if (pressureTrend <= PRESSURE_FALLING_FAST) return "Rain likely soon, take an umbrella";
  if (pressureTrend <= PRESSURE_FALLING_SLOW) return "Showers possible later today";
  if (pressureTrend >= PRESSURE_RISING_FAST)  return "Clearing up, great time outside";
  if (pressureTrend >= PRESSURE_RISING_SLOW)  return "Conditions improving";
  return "Weather looking steady";
}

uint16_t getWeatherColor(float pressureTrend) {
  if (pressureTrend <= PRESSURE_FALLING_FAST) return COLOR_BAD;
  if (pressureTrend <= PRESSURE_FALLING_SLOW) return COLOR_WARN;
  if (pressureTrend >= PRESSURE_RISING_SLOW)  return COLOR_GOOD;
  return COLOR_TEXT;
}

// Picks which icon best represents the current pressure trend
WeatherIcon getWeatherIcon(float pressureTrend) {
  if (pressureTrend <= PRESSURE_FALLING_SLOW) return ICON_RAIN;
  if (pressureTrend >= PRESSURE_RISING_SLOW)  return ICON_SUN;
  return ICON_CLOUD;
}

String getComfortMessage(float tempC, float humidityPct) {
  if (tempC > 28 && humidityPct > 60) return "Hot & humid";
  if (tempC > 28)                     return "It's hot today";
  if (tempC < 15)                     return "Feeling cold";
  if (humidityPct > 70)                return "Quite humid";
  return "Comfortable";
}

String getUVAdvice(float uvIndex) {
  if (uvIndex < 3)  return "UV Low";
  if (uvIndex < 6)  return "UV Moderate";
  if (uvIndex < 8)  return "UV High";
  return "UV Very High";
}

String getLightDescription(float lux) {
  if (lux < 10)   return "Dark";
  if (lux < 200)  return "Dim light";
  if (lux < 1000) return "Bright light";
  return "Daylight";
}

uint16_t getTempColor(float tempC) {
  if (tempC < 15 || tempC > 30) return COLOR_BAD;
  if (tempC < 18 || tempC > 27) return COLOR_WARN;
  return COLOR_GOOD;
}

uint16_t getUVColor(float uvIndex) {
  if (uvIndex >= 8) return COLOR_BAD;
  if (uvIndex >= 3) return COLOR_WARN;
  return COLOR_GOOD;
}

// ---------------------------------------------------------------------------
// Serial Monitor - cleaner aligned table instead of a plain list
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
// Hand-drawn weather icons using basic GFX shapes (no emoji font needed)
// Each is drawn inside roughly a 28x28 box centred at (cx, cy)
// ---------------------------------------------------------------------------
void drawSunIcon(int cx, int cy, uint16_t color) {
  const int radius = 8;
  tft.fillCircle(cx, cy, radius, color);

  // 8 rays radiating outward
  for (int i = 0; i < 8; i++) {
    float angle = i * (PI / 4.0);
    int x1 = cx + (int)((radius + 3) * cos(angle));
    int y1 = cy + (int)((radius + 3) * sin(angle));
    int x2 = cx + (int)((radius + 8) * cos(angle));
    int y2 = cy + (int)((radius + 8) * sin(angle));
    tft.drawLine(x1, y1, x2, y2, color);
  }
}

void drawCloudIcon(int cx, int cy, uint16_t color) {
  tft.fillCircle(cx - 7, cy + 2, 7, color);
  tft.fillCircle(cx + 3, cy - 3, 9, color);
  tft.fillCircle(cx + 12, cy + 2, 6, color);
  tft.fillRect(cx - 13, cy + 2, 32, 8, color);
}

void drawUmbrellaIcon(int cx, int cy, uint16_t color) {
  const int radius = 10;

  // Canopy - top half of a circle (fill circle, then erase the bottom half)
  tft.fillCircle(cx, cy, radius, color);
  tft.fillRect(cx - radius, cy, radius * 2, radius, COLOR_BG);

  // Scalloped bottom edge of the canopy - little notches
  for (int i = -1; i <= 1; i++) {
    tft.fillCircle(cx + (i * (radius / 1.5)), cy, 2, COLOR_BG);
  }

  // Pole
  tft.drawLine(cx, cy, cx, cy + 14, color);

  // Handle hook at the bottom
  tft.drawLine(cx, cy + 14, cx - 4, cy + 14, color);
  tft.drawLine(cx - 4, cy + 14, cx - 4, cy + 10, color);

  // Rain drops falling beside the umbrella for extra clarity
  tft.drawLine(cx + 16, cy - 4, cx + 14, cy + 2, color);
  tft.drawLine(cx + 22, cy - 2, cx + 20, cy + 4, color);
}

void drawWeatherIcon(WeatherIcon icon, int cx, int cy, uint16_t color) {
  switch (icon) {
    case ICON_RAIN:  drawUmbrellaIcon(cx, cy, color); break;
    case ICON_SUN:   drawSunIcon(cx, cy, color);       break;
    case ICON_CLOUD: drawCloudIcon(cx, cy, color);     break;
  }
}

// ---------------------------------------------------------------------------
// drawFittedText - picks the largest text size that still fits the message
// within the available width, so longer sentences stay readable.
// ---------------------------------------------------------------------------
void drawFittedText(String message, int x, int y, uint16_t color, int maxSize) {
  int usableWidth = SCREEN_WIDTH - x - 4;
  int chosenSize = 1;

  for (int size = maxSize; size >= 1; size--) {
    int textWidth = message.length() * CHAR_PIXEL_WIDTH * size;
    if (textWidth <= usableWidth) {
      chosenSize = size;
      break;
    }
  }

  tft.setCursor(x, y);
  tft.setTextSize(chosenSize);
  tft.setTextColor(color);
  tft.println(message);
}

// ---------------------------------------------------------------------------
// TFT Screen 1 - friendly, plain-language summary with a weather icon
// ---------------------------------------------------------------------------
void drawFriendlyScreen(const SensorReading &data, float pressureTrend) {
  tft.fillScreen(COLOR_BG);
  drawHeader("TODAY'S OUTLOOK");

  String      weatherMsg   = getWeatherAdvice(pressureTrend);
  uint16_t    weatherColor = getWeatherColor(pressureTrend);
  WeatherIcon weatherIcon  = getWeatherIcon(pressureTrend);

  // Weather icon on the left, message fitted to the remaining width
  drawWeatherIcon(weatherIcon, 18, 36, weatherColor);
  drawFittedText(weatherMsg, 40, 28, weatherColor, BODY_TEXT_SIZE);

  // Temperature + comfort, combined on one row
  uint16_t tempColor = getTempColor(data.temperatureC);
  tft.setCursor(4, 58);
  tft.setTextSize(BIG_TEXT_SIZE);
  tft.setTextColor(tempColor);
  tft.print(data.temperatureC, 1);
  tft.print("C");

  tft.setCursor(96, 66);
  tft.setTextSize(FOOTER_TEXT_SIZE);
  tft.setTextColor(COLOR_TEXT);
  tft.println(getComfortMessage(data.temperatureC, data.humidityPct));

  // UV line
  uint16_t uvColor = getUVColor(data.uvIndex);
  tft.setCursor(4, 92);
  tft.setTextSize(BODY_TEXT_SIZE);
  tft.setTextColor(uvColor);
  tft.println(getUVAdvice(data.uvIndex));

  // Light line
  tft.setCursor(4, 114);
  tft.setTextSize(FOOTER_TEXT_SIZE);
  tft.setTextColor(COLOR_LABEL);
  tft.println(getLightDescription(data.lightLux));

  // Footer hint
  tft.setCursor(150, 114);
  tft.setTextSize(FOOTER_TEXT_SIZE);
  tft.setTextColor(COLOR_WARN);
  tft.print("D2: raw data ->");
}

// ---------------------------------------------------------------------------
// TFT Screen 2 - raw numeric data
// ---------------------------------------------------------------------------
void drawRawDataScreen(const SensorReading &data, float pressureTrend) {
  tft.fillScreen(COLOR_BG);
  drawHeader("RAW DATA");

  int rowY = 28;
  const int rowHeight = 18;

  auto drawRow = [&](const char* label, String value) {
    tft.setCursor(4, rowY);
    tft.setTextSize(BODY_TEXT_SIZE);
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
  drawRow("Trend:", String(pressureTrend, 2));
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
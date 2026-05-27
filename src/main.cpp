#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

#define SDA_PIN 3
#define SCL_PIN 4

Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;

uint8_t bmpAddress = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting sensors...");

  Wire.begin(SDA_PIN, SCL_PIN);

  // Detect BMP280
  Wire.beginTransmission(0x76);
  if (Wire.endTransmission() == 0) {
    bmpAddress = 0x76;
  }

  Wire.beginTransmission(0x77);
  if (Wire.endTransmission() == 0) {
    bmpAddress = 0x77;
  }

  if (bmpAddress != 0) {
    if (bmp.begin(bmpAddress)) {
      Serial.print("BMP280 found at 0x");
      Serial.println(bmpAddress, HEX);
    } else {
      Serial.println("BMP280 init failed");
    }
  } else {
    Serial.println("BMP280 not detected");
  }

  // Start AHT20
  if (aht.begin()) {
    Serial.println("AHT20 found");
  } else {
    Serial.println("AHT20 not found");
  }

  Serial.println("Setup complete");
}

void loop() {

  // BMP280
  if (bmpAddress != 0) {

    float temp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F;

    Serial.println("BMP280:");
    
    Serial.print("Temp: ");
    Serial.print(temp);
    Serial.println(" C");

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");
  }

  // AHT20
  sensors_event_t humidity;
  sensors_event_t temp;

  aht.getEvent(&humidity, &temp);

  Serial.println("AHT20:");

  Serial.print("Temp: ");
  Serial.print(temp.temperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println(" %");

  Serial.println("-------------------");

  delay(3000);
}
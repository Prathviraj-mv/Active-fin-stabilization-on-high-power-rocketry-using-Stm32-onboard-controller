
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_light.h>

#define LED_PIN PC13

MPU6050 mpu(Wire);
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

// Angle offsets
float angleX_offset = 0;
float angleY_offset = 0;
float angleZ_offset = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();

  // Rapid LED blink for 3 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  digitalWrite(LED_PIN, LOW);

  // Initialize BMP180
  Serial.println("Initializing BMP180...");
  if (!bmp.begin()) {
    Serial.println("BMP180 not found!");
    while (1);
  }
  Serial.println("BMP180 ready.");

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (mpu.begin() != 0) {
    Serial.println("MPU6050 init failed!");
    while (1);
  }
  Serial.println("MPU6050 ready.");
  delay(1000);
  mpu.calcOffsets();  // Sensor offsets

  // === Calibrate Angles to 0 over 2 seconds ===
  Serial.println("Calibrating MPU6050 angles to 0...");
  unsigned long calibStart = millis();
  int count = 0;
  float sumX = 0, sumY = 0, sumZ = 0;

  while (millis() - calibStart < 2000) {
    mpu.update();
    sumX += mpu.getAngleX();
    sumY += mpu.getAngleY();
    sumZ += mpu.getAngleZ();
    count++;
    delay(10);  // Let sensor settle
  }

  angleX_offset = sumX / count;
  angleY_offset = sumY / count;
  angleZ_offset = sumZ / count;

  Serial.println("Calibration complete.");
}

unsigned long lastPrint = 0;
unsigned long lastBlink = 0;

void loop() {
  mpu.update();

  // Print sensor data every 500ms
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    // MPU6050 Angles (Calibrated)
    float angleX = mpu.getAngleX() - angleX_offset;
    float angleY = mpu.getAngleY() - angleY_offset;
    float angleZ = mpu.getAngleZ() - angleZ_offset;

    Serial.print("Angle X: ");
    Serial.print(angleX, 2);
    Serial.print(" | Y: ");
    Serial.print(angleY, 2);
    Serial.print(" | Z: ");
    Serial.print(angleZ, 2);

    // BMP180 data
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
      float temperature;
      bmp.getTemperature(&temperature);

      Serial.print(" | Temp: ");
      Serial.print(temperature);
      Serial.print(" *C");

      Serial.print(" | Pressure: ");
      Serial.print(event.pressure);
      Serial.print(" hPa");

      Serial.print(" | Altitude: ");
      Serial.print(bmp.pressureToAltitude(1013.25, event.pressure));
      Serial.println(" m");
    } else {
      Serial.println("BMP180 error!");
    }
  }

  // LED blink every 5 seconds
  if (millis() - lastBlink >= 5000) {
    lastBlink = millis();
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }
}

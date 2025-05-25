#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>

Servo servo1;
Servo servo2;
MPU6050 mpu;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

const int minAngle = 60;
const int maxAngle = 120;
const int ledPin = PC13;

unsigned long previousLEDTime = 0;
const unsigned long ledInterval = 2000;
bool ledState = false;

float pitchOffset = 0;
float rollOffset = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); 

  // Attach servos and do warm-up movement
  servo1.attach(A0);
  servo2.attach(A1);
  servo1.write(130); delay(500);
  servo1.write(60); delay(500);
  servo1.write(90);  delay(500);

  servo2.write(130); delay(500);
  servo2.write(60); delay(500);
  servo2.write(90);  delay(500);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connected.");
  }

  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("BMP180 not found. Check wiring.");
  } else {
    Serial.println("BMP180 connected.");
  }

  // Calibrate MPU6050 pitch and roll
  Serial.println("Calibrating MPU6050 (2s)...");
  float pitchSum = 0, rollSum = 0;
  int samples = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float roll  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    pitchSum += pitch;
    rollSum += roll;
    samples++;
    delay(10);
  }
  pitchOffset = pitchSum / samples;
  rollOffset = rollSum / samples;
  Serial.print("Pitch Offset: "); Serial.println(pitchOffset);
  Serial.print("Roll Offset: "); Serial.println(rollOffset);

  Serial.println("System ready.");
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - previousLEDTime >= ledInterval) {
    previousLEDTime = currentTime;
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? LOW : HIGH);
  }

  // Read MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI - pitchOffset;
  float roll  = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI - rollOffset;

  int angleOffsetX = constrain(map((int)pitch, -40, 40, -40, 40), -40, 40);
  int angleOffsetY = constrain(map((int)roll,  -40, 40, -40, 40), -40, 40);

  int servo1Angle = constrain(90 + angleOffsetX, minAngle, maxAngle);
  int servo2Angle = constrain(90 + angleOffsetY, minAngle, maxAngle);

  servo1.write(servo1Angle);
  servo2.write(servo2Angle);

  // Read BMP180 sensor
  sensors_event_t event;
  bmp.getEvent(&event);
  float temperature;
  bmp.getTemperature(&temperature);

  Serial.print("Pitch: "); Serial.print(pitch, 1);
  Serial.print(" | Roll: "); Serial.print(roll, 1);
  Serial.print(" | Servo1: "); Serial.print(servo1Angle);
  Serial.print(" | Servo2: "); Serial.print(servo2Angle);

  if (event.pressure) {
    Serial.print(" | Pressure: "); Serial.print(event.pressure);
    Serial.print(" hPa | Temp: "); Serial.print(temperature, 1); Serial.println(" C");
  } else {
    Serial.println(" | BMP180 read error");
  }

  delay(200);
}

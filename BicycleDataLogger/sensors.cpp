#include "sensors.h"
#include "config.h"

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(56, 0x29);
RTC_DS3231 rtc;

bool hasBNO = false;
bool hasMPU = false;
bool hasRTC = false;
bool hasSD  = false;
bool isVibrationMode = false;

void detectSensors() {
  Wire.begin();
  hasBNO = bno1.begin() || bno2.begin();
  hasMPU = false; // implement later
  hasRTC = rtc.begin();
  hasSD  = SD.begin(SD_CS_PIN);
  isVibrationMode = hasMPU && !hasBNO;
}

void setupSensors() {
  if (hasBNO) {
    bno1.setExtCrystalUse(true);
    bno2.setExtCrystalUse(true);
  }
  if (hasRTC) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void readSensors() {
  // Optionally cache data here
}

void logSensors(File &logFile) {
  if (hasRTC) {
    DateTime now = rtc.now();
    logFile.print(now.timestamp());
    logFile.print(",");
  }

  if (hasBNO) {
    imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
    logFile.print(euler1.x()); logFile.print(",");
    logFile.print(euler1.y()); logFile.print(",");
    logFile.print(euler1.z()); logFile.print(",");
  }

  logFile.println();
}

void calibrateSteeringYaw() {
  // Placeholder for future logic
}

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R_PIN, r ? HIGH : LOW);
  digitalWrite(LED_G_PIN, g ? HIGH : LOW);
  digitalWrite(LED_B_PIN, b ? HIGH : LOW);
}

void handleButton() {
  static bool lastButtonState = HIGH;
  static bool currentButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;

  bool reading = digitalRead(MEASURE_BTN_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != currentButtonState) {
      currentButtonState = reading;

      if (currentButtonState == LOW) {
        isMeasuring = !isMeasuring;

        if (isMeasuring && !SD.begin(SD_CS_PIN)) {
          Serial.println("SD init failed.");
          isMeasuring = false;
        }

        Serial.println(isMeasuring ? "STARTED MEASURING" : "STOPPED MEASURING");
        setLED(isMeasuring ? 0 : 1, isMeasuring ? 1 : 0, 0);
      }
    }
  }
  lastButtonState = reading;
}

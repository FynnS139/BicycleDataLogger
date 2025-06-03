#include "config.h"
#include "sensors.h"

void setup() {
  Serial.begin(115200);
  detectSensors();
  setupSensors();
  pinMode(MEASURE_BTN_PIN, INPUT_PULLUP);
  pinMode(FLAG_BTN_PIN, INPUT_PULLUP);
  setLED(1, 0, 0); // Red LED on start
}

void loop() {
  static bool isMeasuring = false;
  static unsigned long lastLogTime = 0;

  handleButton();

  if (isMeasuring && millis() - lastLogTime >= (1000 / DEFAULT_LOG_RATE_HZ)) {
    File logFile = SD.open("/data.csv", FILE_APPEND);
    if (logFile) {
      readSensors();
      logSensors(logFile);
      logFile.close();
    }
    lastLogTime = millis();
  }
}

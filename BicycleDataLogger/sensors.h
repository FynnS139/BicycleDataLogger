#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>

extern Adafruit_BNO055 bno1;
extern Adafruit_BNO055 bno2;
extern RTC_DS3231 rtc;

extern bool hasBNO;
extern bool hasMPU;
extern bool hasRTC;
extern bool hasSD;
extern bool isVibrationMode;

void detectSensors();
void setupSensors();
void readSensors();
void logSensors(File &logFile);
void calibrateSteeringYaw();
void setLED(bool r, bool g, bool b);
void handleButton();

#endif

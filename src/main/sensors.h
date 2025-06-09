#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

void setLED();
void flashLED();
void DangerFlag();
void yawReset();
void waitForIMUCalibration();
void handleButton();
void handleFlagAndYawButton();

#endif

imu::Vector<3> getBikeEulerAngles();
imu::Vector<3> getBikeFrameAcceleration();

extern volatile unsigned long lastPulseTime;
extern volatile unsigned long pulseInterval;
extern volatile unsigned long totalRotations;

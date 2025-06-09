#ifndef CONFIG_H
#define CONFIG_H

// Pin definitions
const int bluePin = 5;
const int greenPin = 6;
const int redPin = 7;
const int chipSelect = 10;
const int hallPin = 2;
const int buttonPin = 4;
const int measureButtonPin = 3;
const int MPU_ADDR = 0x69;

// State flags
bool rtcDetected = false;
bool hallDetected = false;
bool frameDetected = false;
bool forkDetected = false;
bool IMUDetected = false;
bool vibrationIMUDetected = false;
bool isMeasuring = false;
bool isVibrationMode = false;
bool isCalibrated = false;
bool flag = false;
float yawOffset = 0.0;
float yawOffset1 = 0;
float yawOffset2 = 0;
float steeringOffset = 0;
float yawOffsetDisplay = 0;
float pitchOffsetDisplay = 0;
float rollOffsetDisplay = 0;
imu::Quaternion qOffset = imu::Quaternion(1, 0, 0, 0);
float wheelRadius = 0.3556;
float wheelCircumference = 2 * PI * wheelRadius;
float maxSpeedKmh = 25.0;
float maxSpeedMps = maxSpeedKmh / 3.6;
float speedKmh = 0;
float distance = 0;
imu::Quaternion q = bno.getQuat();
float ysqr = q.y() * q.y();
float t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
float t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
float yaw = atan2(t3, t4) * 180.0 / PI;
float y1 = getYawFromQuaternion(bnoFrame) - yawOffset1;
float y2 = getYawFromQuaternion(bnoFork) - yawOffset2;
float yaw1 = getYawFromQuaternion(bnoFrame) - (isCalibrated ? yawOffset1 : 0);
float yaw2 = getYawFromQuaternion(bnoFork) - (isCalibrated ? yawOffset2 : 0);
bool forkDone = false;
bool done = true;
bool calibratedFork = isOrientationCalibrated(bnoFork, "BNO_Fork");
bool calibratedFrame = isOrientationCalibrated(bnoFrame, "BNO_Frame");
float yaw1 = getYawFromQuaternion(bnoFrame) - (isCalibrated ? yawOffset1 : 0);
float yaw2 = getYawFromQuaternion(bnoFork) - (isCalibrated ? yawOffset2 : 0);
float steeringAngle = normalizeAngle(yaw2 - yaw1 - (isCalibrated ? steeringOffset : 0));
float pitch2 = ori1.y();
float roll2 = ori1.z();
float displayYaw = normalizeAngle(yaw1 - yawOffsetDisplay);
float displayPitch = -(pitch2 - pitchOffsetDisplay);
float displayRoll = -(roll2 - rollOffsetDisplay);
imu::Quaternion q = bnoFrame.getQuat();
imu::Quaternion qConj = q.conjugate();
imu::Quaternion accQuat(0, acc.x(), acc.y(), acc.z());
imu::Quaternion rotated = q * accQuat * qConj;
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
bool reading = digitalRead(measureButtonPin);
bool lastButton2State = HIGH;
bool longPress = false;
bool reading = digitalRead(buttonPin);

#endif
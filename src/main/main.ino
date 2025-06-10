// Includes 
#include <Wire.h>
#include <RTClib.h>
#include <SD.h>
File vibrationLogFile;
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Pin definitions
const int bluePin = 5;  // D5
const int greenPin = 6;    // D6
const int redPin = 7; // D7
const int chipSelect = 10;  // SD CS pin
const int hallPin = 2; // D2
const int buttonPin = 4; // D4, flag and zero yaw angle button
const int measureButtonPin = 3; // D3, start stop measuring button

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
imu::Quaternion qOffset = imu::Quaternion(1, 0, 0, 0);  // quaternion

// LED lights setup
void setLED(bool r, bool g, bool b) {
  digitalWrite(redPin, r);
  digitalWrite(greenPin, g);
  digitalWrite(bluePin, b);
}

void flashLED(bool r, bool g, bool b, int times = 11, int delayMs = 200) {
  for (int i = 0; i < times; i++) {
    setLED(r, g, b);
    delay(delayMs);
    setLED(0, 0, 0);
    delay(delayMs);
  }
}

// Flag last 30 seconds of data
void DangerFlag() {
  flashLED(0, 1, 0, 3, 250); // 3 flashes
  flag = true;
  Serial.println("Flagged");
}

// RTC, SD, Hall, Timers
RTC_DS3231 rtc;
const float wheelRadius = 0.3556; // standard swapfiets, but adjust according to bicycle!
const float wheelCircumference = 2 * PI * wheelRadius;
const float maxSpeedKmh = 25.0;
const float maxSpeedMps = maxSpeedKmh / 3.6;
const unsigned long stopTimeout = 4000000;
const unsigned long debounceDelay = 250;
const unsigned long minPulseInterval = wheelCircumference / maxSpeedMps * 1e6;

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile unsigned long totalRotations = 0;
float speedKmh = 0;
float distance = 0;

// Orientation IMUs (BNO055s)
Adafruit_BNO055 bnoFrame = Adafruit_BNO055(55, 0x29); // Frame
Adafruit_BNO055 bnoFork = Adafruit_BNO055(56, 0x28);  // Fork

// Get tilt-compensated yaw from quaternion
float getYawFromQuaternion(Adafruit_BNO055 &bno) {
  imu::Quaternion q = bno.getQuat();
  float ysqr = q.y() * q.y();

  float t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
  float t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());
  float yaw = atan2(t3, t4) * 180.0 / PI;

  return yaw;
}

float normalizeAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

// Reset zero yaw
void yawReset() {
  if (!frameDetected || !forkDetected) {
    Serial.println("one or both IMUs not detected, so yaw reset not possible.");
    return;
  }
  yawOffset1 = getYawFromQuaternion(bnoFrame); //frame bno
  yawOffset2 = getYawFromQuaternion(bnoFork); // fork bno

  float y1 = getYawFromQuaternion(bnoFrame) - yawOffset1; 
  float y2 = getYawFromQuaternion(bnoFork) - yawOffset2; // normalized yaw values
  steeringOffset = normalizeAngle(y2 - y1); 
  
  imu::Vector<3> ori1 = bnoFrame.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawOffsetDisplay = y1;
  pitchOffsetDisplay = ori1.y();
  rollOffsetDisplay = ori1.z();

  isCalibrated = true;
  Serial.print("yaw reset to zero, normalized");
  flashLED(0, 0, 1, 6, 250); // blue flashes 3 seconds
}

// Vibration IMU (MPU6500)
const int MPU_ADDR = 0x69;
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t SMPLRT_DIV = 0x19;
const uint8_t CONFIG = 0x1A;
const uint8_t ACCEL_XOUT_H = 0x3B;

// Sampling settings 
int targetFrequency = 400; // vibration mode frequency
unsigned long sampleIntervalMicros = 1000000UL / targetFrequency;
const unsigned long regularIntervalMicros = 10000; // 1sec/10000 = 100 Hz. - 100 Hz IMU resolution

// functions
float getSteeringAngle() { // steering angle using quaternions
  float yaw1 = getYawFromQuaternion(bnoFrame) - (isCalibrated ? yawOffset1 : 0);
  float yaw2 = getYawFromQuaternion(bnoFork) - (isCalibrated ? yawOffset2 : 0);
  return normalizeAngle(yaw2 - yaw1 - (isCalibrated ? steeringOffset : 0));
}

bool detectRTC() {
  if (rtc.begin()) {
    Serial.println("RTC detected");
    return true;
  }
  Serial.println("RTC not found");
  return false;
}

bool detectHall() {
  pinMode(hallPin, INPUT_PULLUP);
  Serial.println("Hall sensor connected");
  return true;
}

bool detectFrameIMU() {
  if (bnoFrame.begin()) {
    bnoFrame.setExtCrystalUse(true);
    Serial.println("BNO055 Frame detected.");
    return true;
  } else {
    Serial.println("BNO055 Frame NOT detected.");
    return false;
  }
}

bool detectForkIMU() {
  if (bnoFork.begin()) {
    bnoFork.setExtCrystalUse(true);
    Serial.println("BNO055 Fork detected.");
    return true;
  } else {
    Serial.println("BNO055 Fork NOT detected.");
    return false;
  }
}

bool isOrientationCalibrated(Adafruit_BNO055 &bno, const char* label) {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.print(label);
  Serial.print(" Calibration: SYS=");
  Serial.print(sys);
  Serial.print(" GYRO=");
  Serial.print(gyro);
  Serial.print(" ACCEL=");
  Serial.print(accel);
  Serial.print(" MAG=");
  Serial.println(mag);

  return (gyro == 3 && accel == 3 && mag == 3);
}

void waitForIMUCalibration() {
  Serial.println("Waiting for connected BNO055(s) to calibrate: GYRO, ACCEL, and MAG (all = 3)");
  setLED(0,0,1); // blue LED light while calibrating
  bool forkDone = false;
  while (true) {
    bool done = true;

    if (forkDetected) {
      bool calibratedFork = isOrientationCalibrated(bnoFork, "BNO_Fork");
      done &= calibratedFork;
      if (calibratedFork && !forkDone) { // flash blue when fork becomes calibrated
      flashLED(0, 0, 1, 5, 200); // 2 seconds of flashing
      forkDone = true;
      }
    }
    setLED(0, 0, 1); // back to blue LED, until the frame is calibrated

    if (frameDetected) {
      bool calibratedFrame = isOrientationCalibrated(bnoFrame, "BNO_Frame");
      done &= calibratedFrame;
    }

    if (done) {
      Serial.println("All connected BNO055s are calibrated.\n");
      break;
    }
    delay(1000);
  }
  setLED(1,0,0); // red LED light: calibrated but not measuring
}



// Euler angles for frame IMU
imu::Vector<3> getBikeEulerAngles() {
  float yaw1 = getYawFromQuaternion(bnoFrame) - (isCalibrated ? yawOffset1 : 0);
  float yaw2 = getYawFromQuaternion(bnoFork) - (isCalibrated ? yawOffset2 : 0);
  float steeringAngle = normalizeAngle(yaw2 - yaw1 - (isCalibrated ? steeringOffset : 0));

  imu::Vector<3> ori1 = bnoFrame.getVector(Adafruit_BNO055::VECTOR_EULER);
  float pitch2 = ori1.y();
  float roll2 = ori1.z();

  float displayYaw = normalizeAngle(yaw1 - yawOffsetDisplay);
  float displayPitch = -(pitch2 - pitchOffsetDisplay);
  float displayRoll = -(roll2 - rollOffsetDisplay);

  return imu::Vector<3>(displayYaw, displayPitch, displayRoll);
}

// xyz accelerations of frame IMU in the same axis as euler angles frame IMU
imu::Vector<3> getBikeFrameAcceleration() {
  imu::Quaternion q = bnoFrame.getQuat();          // Orientation of frame IMU
  imu::Vector<3> acc = bnoFrame.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  // raw linear acceleration, not yet in bike triad/frame

  imu::Quaternion qConj = q.conjugate();
  imu::Quaternion accQuat(0, acc.x(), acc.y(), acc.z());
  imu::Quaternion rotated = q * accQuat * qConj; // rotate vector into bike frame using q^-1 * acc * q

  return imu::Vector<3>(rotated.x(), rotated.y(), rotated.z());
}

bool detectVibrationIMU() {
  Wire.begin(A4, A5);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x18);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x00);
  Wire.endTransmission(true);

  uint8_t smplrt_div_value = max(1, 1000 / targetFrequency - 1);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(SMPLRT_DIV);
  Wire.write(smplrt_div_value);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6500 detected");
    return true;
  }
  Serial.println("MPU6500 NOT detected");
  return false;
}

void IRAM_ATTR onMagnetPass() {
  if (!isMeasuring || isVibrationMode) return;
  unsigned long currentTime = micros();
  if (currentTime - lastPulseTime < minPulseInterval) return;
  if (lastPulseTime != 0) pulseInterval = currentTime - lastPulseTime;
  lastPulseTime = currentTime;
  totalRotations++; // adds one counter to total rotations for every magnet pass
}

void handleButton() {
  static bool lastButtonState = HIGH;
  static bool currentButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;

  if (isMeasuring) {
    setLED(0,1,0); // green while measuring
  } else {
    setLED(1,0,0); // red while not measuring
  }

  bool reading = digitalRead(measureButtonPin);
  if (reading != lastButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      if (currentButtonState == LOW) {
        isMeasuring = !isMeasuring;
        speedKmh = 0;

      if (isMeasuring) { // FOR TESTING (removing SD card is possible)
        if (!SD.begin(chipSelect)) {
          Serial.println("SD card initialization failed.");
          isMeasuring = false; // prevent logging if SD failed
          return;
        } else {
          Serial.println("SD card initialized.");
      }
      }

        if (isMeasuring && isVibrationMode) {
          vibrationLogFile = SD.open("/Vibrations.csv", FILE_APPEND);
          if (vibrationLogFile) {
            Serial.println("Vibration log file opened.");
          } else {
            Serial.println("Failed to open vibration log file!");
          }
        }

        if (!isMeasuring && isVibrationMode && vibrationLogFile) {
          vibrationLogFile.close();
          Serial.println("Vibration log file closed.");
        }

        Serial.println(isMeasuring ? "STARTED MEASURING" : "STOPPED MEASURING");
        setLED(isMeasuring ? 0 : 1, isMeasuring ? 1 : 0, 0); // Green or Red
      }
    
    }
  }
  lastButtonState = reading;
}

void handleFlagAndYawButton() {
  static bool lastButton2State = HIGH;
  static unsigned long button2PressTime = 0;
  static bool longPress = false; 

  bool reading = digitalRead(buttonPin); // buttonPin = D4

  if (reading == LOW && lastButton2State == HIGH) {
    button2PressTime = millis(); // button just pressed
    longPress = false;
  }

  if (reading == LOW && !longPress) {
    if (millis() - button2PressTime >= 3000) { // 3 seconds
      yawReset();
      Serial.println("Yaw reset triggered");
      longPress = true;
    }
  }

  if (reading == HIGH && lastButton2State == LOW) {
    if (!longPress) {
      DangerFlag();
      Serial.println("Danger flag triggered");
    }
  }
  lastButton2State = reading;
}

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  setLED(0, 0, 1);  // blue
  delay(300);
  setLED(0, 0, 0);  // off
  delay(300); //TEST POWER ON POWER BANK
  Serial.begin(115200);
  unsigned long serialTimeout = millis() + 3000; // if serial is not connected within 3 seconds (so it is connected to a powerbank), then continue without serial monitor
  while (!Serial && millis() < serialTimeout);

  pinMode(measureButtonPin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT); 
  pinMode(bluePin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin), onMagnetPass, FALLING);

  Serial.println("Detecting sensors...");
  setLED(0,0,1); // blue LED light while calibrating
  rtcDetected = detectRTC();
  hallDetected = detectHall();
  frameDetected = detectFrameIMU();
  forkDetected = detectForkIMU(); 
  IMUDetected = frameDetected || forkDetected;
    if (IMUDetected) {
      setLED(0,0,1); // blue LED light while calibrating
      waitForIMUCalibration();
    } else {
      Serial.println("No IMUs (BNO055s) detected");
    }

  vibrationIMUDetected = detectVibrationIMU();
  isVibrationMode = vibrationIMUDetected;
  Serial.print("Vibration mode set to: ");
  Serial.println(isVibrationMode ? "ENABLED" : "DISABLED");

  Serial.println("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card failed.");
    while (1);
  }
  Serial.println("SD card initialized.");
  File logFile = SD.open("/Regular.csv", FILE_WRITE);
  if (logFile && logFile.size() == 0) {
    logFile.println("Timestamp, Angle, Speed, Distance, Yaw, Pitch, Roll, AccX, AccY, AccZ, Flag"); // data columns bike mode  
  }
  logFile.close();
  File vibFile = SD.open("/Vibrations.csv", FILE_WRITE);
  if (vibFile && vibFile.size() == 0) {
    vibFile.println("Timestamp, Accel X, Accel Y, Accel Z, Flag"); // data columns high speed vibration mode
  }
  vibFile.close();
}

void loop() {
  static unsigned long lastLogTime = 0;
  static unsigned long lastVibrationMicros = 0;

  handleButton();
  handleFlagAndYawButton();

  if (!isMeasuring) return;

  unsigned long nowUs = micros();
  unsigned long interval = isVibrationMode ? sampleIntervalMicros : regularIntervalMicros;

  if (nowUs - lastLogTime >= interval) {
    lastLogTime = nowUs;
    unsigned long nowMs = millis();

    float ax = 0, ay = 0, az = 0;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    if (Wire.available() == 6) {
      int16_t rawAx = Wire.read() << 8 | Wire.read();
      int16_t rawAy = Wire.read() << 8 | Wire.read();
      int16_t rawAz = Wire.read() << 8 | Wire.read();
      ax = rawAx / 2048.0;
      ay = rawAy / 2048.0;
      az = rawAz / 2048.0;
    }

    String logLine = "";

    static uint32_t lastRTCSec = 9999; //this is so recorded data is still recorded at e.g. 50Hz, instead of once per second like the rtc
    static String cachedTimestamp = "[waiting]";

    if (rtcDetected) { 
      DateTime now = rtc.now();
      if (now.second() != lastRTCSec) {
        char timestamp[35];  // reserve enough space
        snprintf(timestamp, sizeof(timestamp), "[%04d-%02d-%02d %02d:%02d:%02d]",
              now.year(), now.month(), now.day(),
              now.hour(), now.minute(), now.second());
        cachedTimestamp = timestamp;
        lastRTCSec = now.second();
    }
    logLine += cachedTimestamp;
    } else {
      logLine += "[no RTC]";
    }

    if (isVibrationMode) {
      logLine += String(ax, 2) + ", " + String(ay, 2) + ", " + String(az, 2);
      logLine += flag ? "1" : "0";
      if (vibrationLogFile) {
        vibrationLogFile.println(logLine);
      }
      flag = false;

    } else {
      float steeringAngle = IMUDetected ? getSteeringAngle() : 0;
      float yaw = 0, pitch = 0, roll = 0;
      float accX = 0, accY = 0, accZ = 0;

      if (IMUDetected) {
        imu::Vector<3> euler = getBikeEulerAngles();   
        imu::Vector<3> accFrame = getBikeFrameAcceleration();
          yaw = euler.x();
          pitch = euler.y();
          roll = euler.z();
          accX = accFrame.x();
          accY = accFrame.y();
          accZ = accFrame.z();
          /*
          Serial.print("Frame IMU - Yaw: ");
          Serial.print(euler.x(), 1);
          Serial.print("°, Pitch: ");
          Serial.print(euler.y(), 1);
          Serial.print("°, Roll: ");
          Serial.print(euler.z(), 1);
          Serial.println("°");
          Serial.println("angle:");
          Serial.println(steeringAngle, 1);
          */ // only use for debugging
      }

      noInterrupts();
      unsigned long intervalCopy = pulseInterval;
      unsigned long lastPulseCopy = lastPulseTime;
      interrupts();

     if ((micros() - lastPulseCopy) > stopTimeout) speedKmh = 0;
      else if (intervalCopy > 0) {
        float rotationTimeSec = intervalCopy / 1e6;
        float speedMps = (2 * PI * wheelRadius) / rotationTimeSec;
        distance = totalRotations * wheelCircumference;  // in meters
        speedKmh = speedMps * 3.6;
      }
     // if (isMeasuring && (micros() - lastPulseCopy) > 300000000) { // 5 minutes
     //   isMeasuring = false;
     //   Serial.println("Auto-stopped due to inactivity.");
     //   setLED(1, 0, 0); // Red
     //   if (isVibrationMode && vibrationLogFile) {
     //     vibrationLogFile.close();
     //     Serial.println("Vibration log file closed.");
     //   }
     //
     // }

      logLine += ","; logLine += String(steeringAngle, 2);
      logLine += ","; logLine += String(speedKmh, 2);
      logLine += ","; logLine += String(distance, 2);      
      logLine += ","; logLine += String(yaw, 2);
      logLine += ","; logLine += String(pitch, 2);
      logLine += ","; logLine += String(roll, 2);
      logLine += ","; logLine += String(accX, 2);
      logLine += ","; logLine += String(accY, 2);
      logLine += ","; logLine += String(accZ, 2);    
      logLine += ", " + String(flag ? "1" : "0");

      File logFile = SD.open("/Regular.csv", FILE_APPEND);
      if (logFile) {
        logFile.println(logLine);
        logFile.close();
      }
      flag = false;
    }
  }
}
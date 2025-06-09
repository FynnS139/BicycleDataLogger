#include "config.h"
#include "sensors.h"

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
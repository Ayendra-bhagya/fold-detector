/*
  ESP32 SMART ACTIVITY MONITOR
  ============================

  Hardware:
    - ESP32 Dev Module
    - MPU6050
    - SSD1306 128x64 OLED

  Features:
    - MPU6050 accelerometer + gyroscope
    - Step counting
    - Multi-stage fall detection
    - OLED status display
    - Bluetooth Low Energy (BLE)
    - Real-time JSON telemetry
    - BLE commands:
        RESET_FALL
        RESET_STEPS
        RESET_ALL
        PING
    - Persistent step count using ESP32 Preferences/NVS

  Default wiring:
    ESP32 GPIO21 -> SDA
    ESP32 GPIO22 -> SCL
    3.3V          -> VCC
    GND           -> GND

  Default I2C addresses:
    MPU6050 = 0x68
    OLED    = 0x3C

  Required Arduino libraries:
    - Adafruit MPU6050
    - Adafruit Unified Sensor
    - Adafruit GFX Library
    - Adafruit SSD1306

  The BLE API below uses Arduino String for getValue(),
  which is compatible with ESP32 Arduino BLE versions where:
      BLECharacteristic::getValue()
  returns String.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================
// HARDWARE CONFIGURATION
// ============================================================

static const int I2C_SDA = 21;
static const int I2C_SCL = 22;

static const uint8_t MPU6050_ADDR = 0x68;
static const uint8_t OLED_ADDR = 0x3C;

static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;

// ============================================================
// BLE CONFIGURATION
// ============================================================

static const char* BLE_DEVICE_NAME = "ESP32-Activity-Monitor";

static const char* SERVICE_UUID =
    "7b7a0001-6b3e-4c77-8b2b-0fb0d2bb1001";

static const char* TELEMETRY_UUID =
    "7b7a0002-6b3e-4c77-8b2b-0fb0d2bb1001";

static const char* COMMAND_UUID =
    "7b7a0003-6b3e-4c77-8b2b-0fb0d2bb1001";

// ============================================================
// OBJECTS
// ============================================================

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Preferences preferences;

BLEServer* bleServer = nullptr;
BLECharacteristic* telemetryCharacteristic = nullptr;
BLECharacteristic* commandCharacteristic = nullptr;

bool bleConnected = false;
bool oledAvailable = false;
bool mpuAvailable = false;

// ============================================================
// PERSISTENT DATA
// ============================================================

uint32_t stepCount = 0;

// ============================================================
// SENSOR DATA
// ============================================================

// Acceleration in g
float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

// Gyroscope in degrees/sec
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

float accelerationG = 1.0f;
float smoothedAccelerationG = 1.0f;
float gravityEstimateG = 1.0f;
float dynamicAccelerationG = 0.0f;
float gyroMagnitudeDps = 0.0f;

float orientationBaselineX = 0.0f;
float orientationBaselineY = 0.0f;
float orientationBaselineZ = 1.0f;

float orientationChangeDeg = 0.0f;

// ============================================================
// ACTIVITY STATE
// ============================================================

enum ActivityState {
  ACTIVITY_IDLE,
  ACTIVITY_WALKING,
  ACTIVITY_ACTIVE,
  ACTIVITY_FALL
};

ActivityState activityState = ACTIVITY_IDLE;

const char* activityToString(ActivityState state) {
  switch (state) {
    case ACTIVITY_WALKING:
      return "WALKING";

    case ACTIVITY_ACTIVE:
      return "ACTIVE";

    case ACTIVITY_FALL:
      return "FALL";

    case ACTIVITY_IDLE:
    default:
      return "IDLE";
  }
}

// ============================================================
// STEP DETECTION
// ============================================================

bool stepPulseHigh = false;

float stepPeak = 0.0f;

unsigned long stepPulseStartMs = 0;
unsigned long lastStepMs = 0;

float noiseEstimate = 0.015f;

static const unsigned long MIN_STEP_INTERVAL_MS = 280;
static const unsigned long MIN_STEP_PULSE_MS = 80;

static const float MIN_STEP_PEAK_G = 0.12f;
static const float STEP_LOW_THRESHOLD_G = 0.025f;

// ============================================================
// FALL DETECTION
// ============================================================

bool fallAlert = false;
bool fallCandidate = false;
bool fallEvidence = false;

unsigned long fallCandidateStartMs = 0;
unsigned long lowMovementStartMs = 0;

float impactPeakG = 0.0f;
float fallPeakGyroDps = 0.0f;
float fallPeakOrientationChangeDeg = 0.0f;

// Starting thresholds. These should be calibrated with real testing.
static const float FALL_IMPACT_G = 2.70f;
static const float FALL_GYRO_EVIDENCE_DPS = 180.0f;
static const float FALL_ORIENTATION_EVIDENCE_DEG = 35.0f;

static const float FALL_CONFIRM_ORIENTATION_DEG = 25.0f;

static const float FALL_LOW_ACTIVITY_MIN_G = 0.72f;
static const float FALL_LOW_ACTIVITY_MAX_G = 1.35f;
static const float FALL_LOW_GYRO_MAX_DPS = 85.0f;

static const unsigned long FALL_CONFIRM_WINDOW_MS = 4000;
static const unsigned long FALL_LOW_MOVEMENT_CONFIRM_MS = 500;

// ============================================================
// TIMING
// ============================================================

unsigned long lastSensorSampleMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastOLEDUpdateMs = 0;
unsigned long lastOrientationBaselineMs = 0;

static const unsigned long SENSOR_INTERVAL_MS = 25;      // 40 Hz
static const unsigned long TELEMETRY_INTERVAL_MS = 250;  // 4 Hz
static const unsigned long OLED_INTERVAL_MS = 250;
static const unsigned long BASELINE_UPDATE_INTERVAL_MS = 100;

// ============================================================
// UTILITY FUNCTIONS
// ============================================================

float clampFloat(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }

  if (x > hi) {
    return hi;
  }

  return x;
}

float vectorAngleDeg(
    float x1,
    float y1,
    float z1,
    float x2,
    float y2,
    float z2
) {
  const float mag1 =
      sqrtf(x1 * x1 + y1 * y1 + z1 * z1);

  const float mag2 =
      sqrtf(x2 * x2 + y2 * y2 + z2 * z2);

  if (mag1 < 0.0001f || mag2 < 0.0001f) {
    return 0.0f;
  }

  const float dot =
      (x1 * x2 + y1 * y2 + z1 * z2) / (mag1 * mag2);

  const float safeDot =
      clampFloat(dot, -1.0f, 1.0f);

  return acosf(safeDot) * 180.0f / PI;
}

// ============================================================
// ORIENTATION BASELINE
// ============================================================

void updateOrientationBaseline() {
  const float mag =
      sqrtf(ax * ax + ay * ay + az * az);

  // Only slowly adapt while sensor is approximately still.
  if (mag > 0.88f &&
      mag < 1.12f &&
      gyroMagnitudeDps < 45.0f) {

    float nx = ax / mag;
    float ny = ay / mag;
    float nz = az / mag;

    const float alpha = 0.015f;

    orientationBaselineX =
        (1.0f - alpha) * orientationBaselineX +
        alpha * nx;

    orientationBaselineY =
        (1.0f - alpha) * orientationBaselineY +
        alpha * ny;

    orientationBaselineZ =
        (1.0f - alpha) * orientationBaselineZ +
        alpha * nz;

    const float baseMag =
        sqrtf(
            orientationBaselineX * orientationBaselineX +
            orientationBaselineY * orientationBaselineY +
            orientationBaselineZ * orientationBaselineZ
        );

    if (baseMag > 0.0001f) {
      orientationBaselineX /= baseMag;
      orientationBaselineY /= baseMag;
      orientationBaselineZ /= baseMag;
    }
  }
}

// ============================================================
// STEP THRESHOLD
// ============================================================

float calculateStepThreshold() {
  // Adaptive noise-based threshold.
  const float threshold =
      noiseEstimate * 3.0f + 0.10f;

  return clampFloat(
      threshold,
      0.14f,
      0.30f
  );
}

// ============================================================
// STEP DETECTION
// ============================================================

void updateStepDetection(unsigned long nowMs) {
  const float absDynamic =
      fabsf(dynamicAccelerationG);

  // Learn noise only outside a current step pulse.
  if (!stepPulseHigh) {
    const float noiseAlpha = 0.02f;

    noiseEstimate =
        (1.0f - noiseAlpha) * noiseEstimate +
        noiseAlpha * clampFloat(
            absDynamic,
            0.0f,
            0.5f
        );
  }

  const float stepThreshold =
      calculateStepThreshold();

  // Start a potential step.
  if (!stepPulseHigh) {
    if ((nowMs - lastStepMs) >= MIN_STEP_INTERVAL_MS &&
        dynamicAccelerationG > stepThreshold) {

      stepPulseHigh = true;
      stepPulseStartMs = nowMs;
      stepPeak = dynamicAccelerationG;
    }

    return;
  }

  // Track peak during step pulse.
  if (dynamicAccelerationG > stepPeak) {
    stepPeak = dynamicAccelerationG;
  }

  // End step pulse after signal returns near baseline.
  if (dynamicAccelerationG < STEP_LOW_THRESHOLD_G &&
      (nowMs - stepPulseStartMs) >= MIN_STEP_PULSE_MS) {

    if (stepPeak >= MIN_STEP_PEAK_G &&
        (nowMs - lastStepMs) >= MIN_STEP_INTERVAL_MS) {

      stepCount++;

      // Save current step count to NVS.
      preferences.putUInt(
          "steps",
          stepCount
      );

      lastStepMs = nowMs;
    }

    stepPulseHigh = false;
    stepPeak = 0.0f;
  }

  // Safety timeout.
  if ((nowMs - stepPulseStartMs) > 1000) {
    stepPulseHigh = false;
    stepPeak = 0.0f;
  }
}

// ============================================================
// FALL CANDIDATE
// ============================================================

void startFallCandidate(unsigned long nowMs) {
  fallCandidate = true;
  fallEvidence = false;

  fallCandidateStartMs = nowMs;
  lowMovementStartMs = 0;

  impactPeakG = accelerationG;
  fallPeakGyroDps = gyroMagnitudeDps;
  fallPeakOrientationChangeDeg = orientationChangeDeg;
}

// ============================================================
// FALL DETECTION
// ============================================================

void updateFallDetection(unsigned long nowMs) {
  // Alarm remains latched until user resets it.
  if (fallAlert) {
    activityState = ACTIVITY_FALL;
    return;
  }

  // Initial high-impact event.
  if (!fallCandidate &&
      accelerationG >= FALL_IMPACT_G) {

    startFallCandidate(nowMs);
    return;
  }

  if (!fallCandidate) {
    return;
  }

  // Track peaks.
  if (accelerationG > impactPeakG) {
    impactPeakG = accelerationG;
  }

  if (gyroMagnitudeDps > fallPeakGyroDps) {
    fallPeakGyroDps = gyroMagnitudeDps;
  }

  if (orientationChangeDeg > fallPeakOrientationChangeDeg) {
    fallPeakOrientationChangeDeg =
        orientationChangeDeg;
  }

  const bool gyroEvidence =
      fallPeakGyroDps >= FALL_GYRO_EVIDENCE_DPS;

  const bool orientationEvidence =
      fallPeakOrientationChangeDeg >=
      FALL_ORIENTATION_EVIDENCE_DEG;

  if (gyroEvidence || orientationEvidence) {
    fallEvidence = true;
  }

  // Post-impact low-movement state.
  const bool lowMovement =
      accelerationG >= FALL_LOW_ACTIVITY_MIN_G &&
      accelerationG <= FALL_LOW_ACTIVITY_MAX_G &&
      gyroMagnitudeDps <= FALL_LOW_GYRO_MAX_DPS;

  if (lowMovement) {
    if (lowMovementStartMs == 0) {
      lowMovementStartMs = nowMs;
    }
  } else {
    lowMovementStartMs = 0;
  }

  const bool sustainedInactivity =
      lowMovementStartMs != 0 &&
      (nowMs - lowMovementStartMs) >=
      FALL_LOW_MOVEMENT_CONFIRM_MS;

  const bool changedPosture =
      fallPeakOrientationChangeDeg >=
      FALL_CONFIRM_ORIENTATION_DEG;

  // Confirm fall:
  //   1. Strong impact
  //   2. Gyro/orientation evidence
  //   3. Low movement afterwards
  //   4. Meaningful posture change
  if (fallEvidence &&
      sustainedInactivity &&
      changedPosture) {

    fallAlert = true;
    activityState = ACTIVITY_FALL;

    preferences.putBool(
        "fall",
        true
    );

    return;
  }

  // Candidate timed out.
  if ((nowMs - fallCandidateStartMs) >
      FALL_CONFIRM_WINDOW_MS) {

    fallCandidate = false;
    fallEvidence = false;

    lowMovementStartMs = 0;

    impactPeakG = 0.0f;
    fallPeakGyroDps = 0.0f;
    fallPeakOrientationChangeDeg = 0.0f;
  }
}

// ============================================================
// ACTIVITY STATE
// ============================================================

void updateActivityState(unsigned long nowMs) {
  if (fallAlert) {
    activityState = ACTIVITY_FALL;
    return;
  }

  const bool recentStep =
      lastStepMs != 0 &&
      (nowMs - lastStepMs) < 2000;

  if (recentStep) {
    activityState = ACTIVITY_WALKING;
    return;
  }

  const bool activeMotion =
      fabsf(dynamicAccelerationG) > 0.08f ||
      gyroMagnitudeDps > 60.0f;

  if (activeMotion) {
    activityState = ACTIVITY_ACTIVE;
  } else {
    activityState = ACTIVITY_IDLE;
  }
}

// ============================================================
// BLE SERVER CALLBACKS
// ============================================================

class ServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer* server) override {
    bleConnected = true;

    Serial.println(
        "BLE client connected."
    );
  }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;

    Serial.println(
        "BLE client disconnected."
    );

    // Start advertising again.
    delay(100);
    BLEDevice::startAdvertising();
  }
};

// ============================================================
// BLE COMMAND CALLBACKS
// ============================================================

class CommandCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* characteristic) override {

    // IMPORTANT:
    // Current ESP32 BLE API version returns Arduino String.
    String value = characteristic->getValue();

    if (value.length() == 0) {
      return;
    }

    String command = value;

    command.trim();
    command.toUpperCase();

    Serial.print("BLE command received: ");
    Serial.println(command);

    // --------------------------------------------------------
    // RESET FALL
    // --------------------------------------------------------
    if (command == "RESET_FALL") {

      fallAlert = false;
      fallCandidate = false;
      fallEvidence = false;

      lowMovementStartMs = 0;

      impactPeakG = 0.0f;
      fallPeakGyroDps = 0.0f;
      fallPeakOrientationChangeDeg = 0.0f;

      preferences.putBool(
          "fall",
          false
      );

      activityState = ACTIVITY_IDLE;

      Serial.println(
          "Fall alert reset."
      );
    }

    // --------------------------------------------------------
    // RESET STEPS
    // --------------------------------------------------------
    else if (command == "RESET_STEPS") {

      stepCount = 0;

      preferences.putUInt(
          "steps",
          stepCount
      );

      lastStepMs = millis();

      stepPulseHigh = false;
      stepPeak = 0.0f;

      Serial.println(
          "Step count reset."
      );
    }

    // --------------------------------------------------------
    // RESET ALL
    // --------------------------------------------------------
    else if (command == "RESET_ALL") {

      stepCount = 0;

      fallAlert = false;
      fallCandidate = false;
      fallEvidence = false;

      preferences.putUInt(
          "steps",
          0
      );

      preferences.putBool(
          "fall",
          false
      );

      lastStepMs = millis();

      stepPulseHigh = false;
      stepPeak = 0.0f;

      lowMovementStartMs = 0;

      impactPeakG = 0.0f;
      fallPeakGyroDps = 0.0f;
      fallPeakOrientationChangeDeg = 0.0f;

      activityState = ACTIVITY_IDLE;

      Serial.println(
          "All data reset."
      );
    }

    // --------------------------------------------------------
    // PING
    // --------------------------------------------------------
    else if (command == "PING") {

      Serial.println(
          "PING received."
      );
    }
  }
};

// ============================================================
// BLE SETUP
// ============================================================

void setupBLE() {
  BLEDevice::init(
      BLE_DEVICE_NAME
  );

  bleServer =
      BLEDevice::createServer();

  bleServer->setCallbacks(
      new ServerCallbacks()
  );

  BLEService* service =
      bleServer->createService(
          SERVICE_UUID
      );

  // Telemetry:
  // READ + NOTIFY
  telemetryCharacteristic =
      service->createCharacteristic(
          TELEMETRY_UUID,
          BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY
      );

  telemetryCharacteristic->addDescriptor(
      new BLE2902()
  );

  // Command:
  // WRITE + WRITE WITHOUT RESPONSE
  commandCharacteristic =
      service->createCharacteristic(
          COMMAND_UUID,
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR
      );

  commandCharacteristic->setCallbacks(
      new CommandCallbacks()
  );

  telemetryCharacteristic->setValue(
      "{\"status\":\"ready\"}"
  );

  service->start();

  BLEAdvertising* advertising =
      BLEDevice::getAdvertising();

  advertising->addServiceUUID(
      SERVICE_UUID
  );

  advertising->setScanResponse(true);

  advertising->setMinPreferred(
      0x06
  );

  advertising->setMinPreferred(
      0x12
  );

  BLEDevice::startAdvertising();

  Serial.println(
      "BLE advertising started."
  );

  Serial.print(
      "BLE device name: "
  );

  Serial.println(
      BLE_DEVICE_NAME
  );
}

// ============================================================
// MPU6050 SETUP
// ============================================================

void setupMPU6050() {
  if (!mpu.begin(
          MPU6050_ADDR,
          &Wire
      )) {

    Serial.println(
        "ERROR: MPU6050 not found!"
    );

    mpuAvailable = false;
    return;
  }

  mpuAvailable = true;

  mpu.setAccelerometerRange(
      MPU6050_RANGE_8_G
  );

  mpu.setGyroRange(
      MPU6050_RANGE_500_DEG
  );

  mpu.setFilterBandwidth(
      MPU6050_BAND_21_HZ
  );

  Serial.println(
      "MPU6050 initialized."
  );
}

// ============================================================
// SENSOR READING
// ============================================================

void readSensors() {
  if (!mpuAvailable) {
    return;
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  mpu.getEvent(
      &accel,
      &gyro,
      &temp
  );

  const float G = 9.80665f;

  // Acceleration -> g
  ax =
      accel.acceleration.x / G;

  ay =
      accel.acceleration.y / G;

  az =
      accel.acceleration.z / G;

  // Gyroscope -> degrees/sec
  gx =
      gyro.gyro.x * 180.0f / PI;

  gy =
      gyro.gyro.y * 180.0f / PI;

  gz =
      gyro.gyro.z * 180.0f / PI;

  // Acceleration magnitude.
  accelerationG =
      sqrtf(
          ax * ax +
          ay * ay +
          az * az
      );

  // Fast smoothing.
  const float smoothAlpha = 0.22f;

  smoothedAccelerationG =
      (1.0f - smoothAlpha) *
      smoothedAccelerationG +
      smoothAlpha *
      accelerationG;

  // Slow gravity estimation.
  const float gravityAlpha = 0.015f;

  gravityEstimateG =
      (1.0f - gravityAlpha) *
      gravityEstimateG +
      gravityAlpha *
      smoothedAccelerationG;

  // Dynamic acceleration.
  dynamicAccelerationG =
      smoothedAccelerationG -
      gravityEstimateG;

  // Gyroscope magnitude.
  gyroMagnitudeDps =
      sqrtf(
          gx * gx +
          gy * gy +
          gz * gz
      );

  // Orientation difference from baseline.
  orientationChangeDeg =
      vectorAngleDeg(
          ax,
          ay,
          az,
          orientationBaselineX,
          orientationBaselineY,
          orientationBaselineZ
      );
}

// ============================================================
// OLED HELPERS
// ============================================================

void oledCenteredText(
    const String& text,
    int y,
    uint8_t size
) {
  display.setTextSize(size);
  display.setTextColor(
      SSD1306_WHITE
  );

  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(
      text.c_str(),
      0,
      y,
      &x1,
      &y1,
      &w,
      &h
  );

  const int x =
      (SCREEN_WIDTH - (int)w) / 2;

  display.setCursor(
      x,
      y
  );

  display.print(text);
}

// ============================================================
// OLED UPDATE
// ============================================================

void updateOLED() {
  if (!oledAvailable) {
    return;
  }

  display.clearDisplay();
  display.setTextColor(
      SSD1306_WHITE
  );

  // ----------------------------------------------------------
  // FALL ALERT SCREEN
  // ----------------------------------------------------------
  if (fallAlert) {

    display.setTextSize(1);

    display.setCursor(
        0,
        0
    );

    display.println(
        F("!!! FALL ALERT !!!")
    );

    oledCenteredText(
        "FALL",
        17,
        2
    );

    display.setTextSize(1);

    display.setCursor(
        0,
        42
    );

    display.print(
        F("Check user now")
    );

    display.setCursor(
        0,
        54
    );

    display.print(
        F("BLE: ")
    );

    display.print(
        bleConnected
            ? F("CONNECTED")
            : F("OFFLINE")
    );

    display.display();

    return;
  }

  // ----------------------------------------------------------
  // NORMAL SCREEN
  // ----------------------------------------------------------

  display.setTextSize(1);

  display.setCursor(
      0,
      0
  );

  display.println(
      F("SMART ACTIVITY")
  );

  display.setCursor(
      0,
      12
  );

  display.print(
      F("Steps: ")
  );

  display.println(
      stepCount
  );

  display.setCursor(
      0,
      24
  );

  display.print(
      F("Activity: ")
  );

  display.println(
      activityToString(
          activityState
      )
  );

  display.setCursor(
      0,
      36
  );

  display.print(
      F("Acc: ")
  );

  display.print(
      accelerationG,
      2
  );

  display.println(
      F(" g")
  );

  display.setCursor(
      0,
      48
  );

  display.print(
      F("BLE: ")
  );

  display.println(
      bleConnected
          ? F("CONNECTED")
          : F("ADVERTISING")
  );

  display.display();
}

// ============================================================
// JSON TELEMETRY
// ============================================================

String makeTelemetryJSON() {
  String json;

  json.reserve(
      750
  );

  json += F("{");

  json += F("\"steps\":");
  json += String(stepCount);

  json += F(",\"fall\":");
  json += fallAlert
      ? F("true")
      : F("false");

  json += F(",\"fallCandidate\":");
  json += fallCandidate
      ? F("true")
      : F("false");

  json += F(",\"activity\":\"");
  json += activityToString(
      activityState
  );
  json += F("\"");

  json += F(",\"ax\":");
  json += String(
      ax,
      3
  );

  json += F(",\"ay\":");
  json += String(
      ay,
      3
  );

  json += F(",\"az\":");
  json += String(
      az,
      3
  );

  json += F(",\"gx\":");
  json += String(
      gx,
      2
  );

  json += F(",\"gy\":");
  json += String(
      gy,
      2
  );

  json += F(",\"gz\":");
  json += String(
      gz,
      2
  );

  json += F(",\"accG\":");
  json += String(
      accelerationG,
      3
  );

  json += F(",\"gyroDps\":");
  json += String(
      gyroMagnitudeDps,
      1
  );

  json += F(",\"orientationChange\":");
  json += String(
      orientationChangeDeg,
      1
  );

  json += F(",\"impactPeakG\":");
  json += String(
      impactPeakG,
      2
  );

  json += F(",\"ble\":");
  json += bleConnected
      ? F("true")
      : F("false");

  json += F(",\"uptime\":");
  json += String(
      millis()
  );

  json += F("}");

  return json;
}

// ============================================================
// SEND BLE TELEMETRY
// ============================================================

void sendTelemetry() {
  if (!bleConnected ||
      telemetryCharacteristic == nullptr) {

    return;
  }

  String json =
      makeTelemetryJSON();

  telemetryCharacteristic->setValue(
      json.c_str()
  );

  telemetryCharacteristic->notify();
}

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
      115200
  );

  delay(500);

  Serial.println();
  Serial.println(
      F("======================================")
  );

  Serial.println(
      F(" ESP32 SMART ACTIVITY MONITOR")
  );

  Serial.println(
      F("======================================")
  );

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
      I2C_SDA,
      I2C_SCL
  );

  Wire.setClock(
      400000
  );

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  if (display.begin(
          SSD1306_SWITCHCAPVCC,
          OLED_ADDR
      )) {

    oledAvailable = true;

    display.clearDisplay();

    display.setTextColor(
        SSD1306_WHITE
    );

    oledCenteredText(
        "ESP32",
        8,
        2
    );

    oledCenteredText(
        "ACTIVITY",
        31,
        1
    );

    oledCenteredText(
        "MONITOR",
        43,
        1
    );

    display.display();

    delay(1200);

    Serial.println(
        "OLED initialized."
    );

  } else {

    oledAvailable = false;

    Serial.println(
        "WARNING: OLED not found."
    );
  }

  // ----------------------------------------------------------
  // MPU6050
  // ----------------------------------------------------------

  setupMPU6050();

  // ----------------------------------------------------------
  // PERSISTENT STORAGE
  // ----------------------------------------------------------

  preferences.begin(
      "activity",
      false
  );

  stepCount =
      preferences.getUInt(
          "steps",
          0
      );

  fallAlert =
      preferences.getBool(
          "fall",
          false
      );

  // ----------------------------------------------------------
  // INITIAL MPU SAMPLE
  // ----------------------------------------------------------

  if (mpuAvailable) {

    readSensors();

    const float mag =
        sqrtf(
            ax * ax +
            ay * ay +
            az * az
        );

    if (mag > 0.0001f) {

      orientationBaselineX =
          ax / mag;

      orientationBaselineY =
          ay / mag;

      orientationBaselineZ =
          az / mag;
    }

    smoothedAccelerationG =
        accelerationG;

    gravityEstimateG =
        accelerationG;
  }

  // ----------------------------------------------------------
  // BLE
  // ----------------------------------------------------------

  setupBLE();

  Serial.println();

  Serial.println(
      F("System ready.")
  );

  Serial.print(
      F("Saved steps: ")
  );

  Serial.println(
      stepCount
  );

  Serial.print(
      F("Saved fall state: ")
  );

  Serial.println(
      fallAlert
          ? F("ACTIVE")
          : F("NORMAL")
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  const unsigned long now =
      millis();

  // ----------------------------------------------------------
  // SENSOR
  // ----------------------------------------------------------

  if (mpuAvailable &&
      (now - lastSensorSampleMs >=
       SENSOR_INTERVAL_MS)) {

    lastSensorSampleMs =
        now;

    readSensors();

    if (now -
        lastOrientationBaselineMs >=
        BASELINE_UPDATE_INTERVAL_MS) {

      lastOrientationBaselineMs =
          now;

      updateOrientationBaseline();
    }

    updateStepDetection(
        now
    );

    updateFallDetection(
        now
    );

    updateActivityState(
        now
    );
  }

  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  if (now -
      lastOLEDUpdateMs >=
      OLED_INTERVAL_MS) {

    lastOLEDUpdateMs =
        now;

    updateOLED();
  }

  // ----------------------------------------------------------
  // BLE TELEMETRY
  // ----------------------------------------------------------

  if (now -
      lastTelemetryMs >=
      TELEMETRY_INTERVAL_MS) {

    lastTelemetryMs =
        now;

    sendTelemetry();
  }

  delay(1);
}

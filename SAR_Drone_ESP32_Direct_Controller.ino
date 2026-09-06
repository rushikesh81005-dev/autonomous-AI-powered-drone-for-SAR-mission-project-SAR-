/*
 * SAR Drone - ESP32 Dev Module (38 pin), direct Quad-X controller
 * --------------------------------------------------------------------------
 * Target hardware:
 *   - Classic ESP32 Dev Module, 38 pin
 *   - MPU6050 on I2C (GPIO 21/22)
 *   - GPS on UART2 (GPIO 16/17)
 *   - OV7670-style 8-bit parallel grayscale camera
 *   - Four standard PWM ESCs (assumed 1000-2000 us, 50 Hz)
 *
 * IMPORTANT SAFETY LIMITS
 *   1. BENCH_TEST_MODE is deliberately true. In this mode every physical ESC
 *      output stays at ESC_MIN_US even though the controller computes a mix.
 *   2. Remove all propellers for calibration, motor-order and sign testing.
 *   3. An MPU6050 + GPS cannot provide dependable altitude hold. There is no
 *      autonomous takeoff/landing in this build. THR is an operator-set,
 *      open-loop throttle and must never be treated as altitude control.
 *   4. There is no absolute heading sensor. Yaw drifts while hovering. GPS
 *      course can correct yaw only while the aircraft is already moving.
 *   5. Camera analysis reports a VISUAL CANDIDATE, not a verified human.
 *
 * This sketch is a bench-development controller. It is not certified or
 * field-ready rescue-aircraft software.
 * Revision: 1.1 - second critical review, 2026-09-04
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "esp_camera.h"
#include <math.h>
#include <ctype.h>
#include <string.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// ============================= BUILD SAFETY ================================

static constexpr bool BENCH_TEST_MODE          = true;
static constexpr bool ENABLE_CAMERA            = true;
static constexpr bool ENABLE_GESTURE_COMMANDS  = false;
static constexpr bool ENABLE_VIBRATION_RTH     = false;
static constexpr bool ENABLE_WIFI_OTA          = false;
static constexpr bool ENABLE_UDP_TELEMETRY     = false;
static constexpr bool ENABLE_BATTERY_MONITOR   = false;
static constexpr bool ENABLE_VOICE_UART        = false;

// The classic 38-pin board has no spare ADC1 pin with this parallel camera.
// Battery monitoring therefore remains disabled until an external I2C ADC
// (recommended) or a revised, electrically validated pin map is installed.

// =============================== PIN MAP ====================================

static constexpr uint8_t MPU_ADDR          = 0x68;
static constexpr int I2C_SDA_PIN           = 21;
static constexpr int I2C_SCL_PIN           = 22;
static constexpr int GPS_RX_PIN            = 16;
static constexpr int GPS_TX_PIN            = 17;
static constexpr int MISSION_SWITCH_PIN    = 23; // LOW permits arming

/* Quad-X motor order, viewed from above:
 *          FRONT
 *     M1 (CCW)   M2 (CW)
 *     M4 (CW)    M3 (CCW)
 */
static constexpr int MOTOR_1_PIN = 25; // front-left
static constexpr int MOTOR_2_PIN = 26; // front-right
static constexpr int MOTOR_3_PIN = 27; // rear-right
static constexpr int MOTOR_4_PIN = 19; // rear-left

// Camera pin map from the supplied sketch.
static constexpr int CAM_D0    = 34;
static constexpr int CAM_D1    = 35;
static constexpr int CAM_D2    = 36;
static constexpr int CAM_D3    = 39;
static constexpr int CAM_D4    = 32;
static constexpr int CAM_D5    = 33;
static constexpr int CAM_D6    = 14;
static constexpr int CAM_D7    = 13;
static constexpr int CAM_XCLK  = 4;
static constexpr int CAM_PCLK  = 18;
static constexpr int CAM_VSYNC = 5;
static constexpr int CAM_HREF  = 15;

// GPIO 4 cannot simultaneously be camera XCLK and voice UART RX. Voice UART
// is therefore disabled for this exact board/pin map. Commands remain
// available over USB Serial and may later be forwarded by a voice base station.

// ============================= ESC / CONTROL ================================

static constexpr uint16_t ESC_MIN_US       = 1000;
static constexpr uint16_t ESC_IDLE_US      = 1060;
static constexpr uint16_t ESC_MAX_US       = 1900;
static constexpr uint16_t THROTTLE_MAX_US  = 1750; // deliberate first-stage cap
static constexpr uint16_t ESC_PWM_HZ       = 50;
static constexpr uint8_t  ESC_PWM_BITS     = 16;
static constexpr uint8_t  MOTOR_CHANNEL[4] = {4, 5, 6, 7}; // avoids camera ch 0

// Sensor orientation assumption: MPU X points forward, Y right, Z upward.
// Change only after a propeller-off axis test. The signs must make the displayed
// angle/rate increase in the documented direction.
static constexpr float ACCEL_X_SIGN =  1.0f;
static constexpr float ACCEL_Y_SIGN =  1.0f;
static constexpr float ACCEL_Z_SIGN =  1.0f;
static constexpr float GYRO_X_SIGN  =  1.0f;
static constexpr float GYRO_Y_SIGN  =  1.0f;
static constexpr float GYRO_Z_SIGN  = -1.0f; // yaw positive clockwise for GPS

static constexpr uint32_t CONTROL_PERIOD_US = 4000; // 250 Hz
static constexpr float CONTROL_DT           = 0.004f;
static constexpr float COMPLEMENTARY_ALPHA  = 0.985f;
static constexpr float MAX_TILT_DEG          = 55.0f;
static constexpr float EMERGENCY_TILT_DEG    = 75.0f;
static constexpr float MAX_ANGLE_RATE_DPS    = 180.0f;

// Open-loop navigation values. These do NOT create altitude hold.
static constexpr float NAV_FORWARD_PITCH_DEG = -6.0f;
static constexpr float NAV_MAX_ROLL_DEG      = 8.0f;
static constexpr float WAYPOINT_RADIUS_M     = 8.0f;
static constexpr float GPS_COURSE_MIN_MPS    = 1.5f;
static constexpr uint32_t GPS_TIMEOUT_MS      = 2500;

// ============================== NETWORK =====================================

static const char *WIFI_SSID       = "DRONE_BASE_STATION";
static const char *WIFI_PASSWORD   = "CHANGE_THIS_WIFI_PASSWORD";
static const char *OTA_PASSWORD    = "CHANGE_THIS_OTA_PASSWORD";
static const IPAddress BASE_IP(192, 168, 4, 2);
static constexpr uint16_t TELEMETRY_PORT = 14560;

WiFiUDP telemetryUdp;
volatile bool otaServiceReady = false;
volatile bool udpServiceReady = false;

// =============================== TYPES ======================================

enum class MissionState : uint8_t {
  DISARMED,
  STABILIZE,
  FLY_TO_WAYPOINT,
  RETURN_HOME,
  OBSTACLE_BRAKE,
  SURVEY_GRID,
  GPS_FAILSAFE_HOLD,
  EMERGENCY_STOP
};

struct GPSData {
  double lat = 0.0;
  double lng = 0.0;
  float speedMps = 0.0f;
  float courseDeg = 0.0f;
  uint8_t satellites = 0;
  bool fix = false;
  bool courseValid = false;
  uint32_t lastFixMs = 0;
};

struct AttitudeData {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  float gxDps = 0.0f;
  float gyDps = 0.0f;
  float gzDps = 0.0f;
};

struct PID {
  float kp;
  float ki;
  float kd;
  float integral;
  float previousMeasurement;
  float outputLimit;

  float update(float setpoint, float measurement, float dt) {
    const float error = setpoint - measurement;
    integral += error * dt;
    if (ki > 0.00001f) {
      const float iLimit = outputLimit / ki;
      integral = constrain(integral, -iLimit, iLimit);
    }
    // Derivative on measurement avoids a large kick when setpoint changes.
    const float derivative = -(measurement - previousMeasurement) / dt;
    previousMeasurement = measurement;
    return constrain(kp * error + ki * integral + kd * derivative,
                     -outputLimit, outputLimit);
  }

  void reset(float measurement = 0.0f) {
    integral = 0.0f;
    previousMeasurement = measurement;
  }
};

struct SurveyPlan {
  bool valid = false;
  double originLat = 0.0;
  double originLng = 0.0;
  float sideKm = 0.0f;
  float spacingKm = 0.0f;
  uint16_t pointsPerLine = 0;
  uint16_t lineCount = 0;
  uint32_t waypointIndex = 0;
  uint32_t waypointCount = 0;
};

// ============================= SHARED STATE =================================

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile MissionState mission = MissionState::DISARMED;
volatile bool motorsArmed = false;
volatile bool imuHealthy = false;
volatile bool imuCalibrated = false;
volatile bool cameraHealthy = false;
volatile bool obstacleBrakeRequested = false;
volatile bool visualCandidateConfirmed = false;
volatile bool vibrationAlert = false;
volatile float vibrationRmsG = 0.0f;
volatile bool calibrationRequested = false;
volatile bool yawResetRequested = false;
volatile float requestedYawDeg = 0.0f;

GPSData gps;
AttitudeData attitude;
SurveyPlan survey;

double homeLat = 0.0;
double homeLng = 0.0;
volatile bool homeLocked = false;
double targetLat = 0.0;
double targetLng = 0.0;
volatile bool targetValid = false;
volatile bool headingReferenceSet = false;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float commandedThrottleUs = ESC_MIN_US;
float desiredRollDeg = 0.0f;
float desiredPitchDeg = 0.0f;
float desiredYawRateDps = 0.0f;
uint16_t computedMotorUs[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};

// Angle outer-loop proportional gains: angle error -> desired angular rate.
float rollAngleKp  = 4.0f;
float pitchAngleKp = 4.0f;

// Conservative STARTING values only. Every airframe requires tuning.
PID rollRatePID  = {1.20f, 0.35f, 0.018f, 0, 0, 260};
PID pitchRatePID = {1.20f, 0.35f, 0.018f, 0, 0, 260};
PID yawRatePID   = {1.00f, 0.20f, 0.000f, 0, 0, 160};

TaskHandle_t flightTaskHandle = nullptr;
TaskHandle_t cameraTaskHandle = nullptr;
TaskHandle_t serviceTaskHandle = nullptr;

// ========================== FUNCTION DECLARATIONS ===========================

void flightTask(void *parameter);
void cameraTask(void *parameter);
void serviceTask(void *parameter);

bool initializeMPU();
bool calibrateMPU(uint16_t samples = 1200);
bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
void updateAttitude(float ax, float ay, float az, float gx, float gy, float gz);
void updateVibrationMonitor(float ax, float ay, float az);

void initializeEscOutputs();
void writeEscMicroseconds(uint8_t motor, uint16_t pulseUs);
void writeAllMotorsMinimum();
void runFlightController();
void computeMotorMix(float throttleUs, float rollCorrection,
                     float pitchCorrection, float yawCorrection);
void resetControllers();
bool armMotors();
void disarmMotors(const char *reason);

void parseGPS();
bool parseNMEALine(char *line);
bool validateNMEAChecksum(const char *line);
double nmeaCoordinateToDegrees(const char *field, bool longitude);
double distanceMeters(double lat1, double lon1, double lat2, double lon2);
float bearingDegrees(double lat1, double lon1, double lat2, double lon2);
float wrap180(float angle);
float wrap360(float angle);

void updateMissionLogic();
void updateNavigationSetpoints();
bool configureSurvey(double originLat, double originLng,
                     float sideKm, float spacingKm);
bool surveyWaypoint(uint32_t index, double &lat, double &lng);
void advanceSurveyWaypoint();

bool initializeCamera();
uint8_t frameLuminance(const camera_fb_t *frame, int pixelIndex);
void analyzeCameraFrame(camera_fb_t *frame);
void reportVisualCandidate();

void processSerialCommands();
void printStatus();
void printHelp();
void setupNetworkServices();
void sendTelemetry();

// ================================ SETUP ======================================

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println();
  Serial.println("SAR ESP32 direct controller starting...");
  Serial.printf("BENCH_TEST_MODE=%s\n", BENCH_TEST_MODE ? "TRUE" : "FALSE");

  pinMode(MISSION_SWITCH_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  if (!initializeMPU()) {
    Serial.println("FATAL: MPU6050 not detected. Motors remain disabled.");
  } else {
    Serial.println("Keep the aircraft level and completely still: calibrating MPU6050...");
    imuCalibrated = calibrateMPU();
    Serial.println(imuCalibrated ? "MPU6050 calibration complete."
                                 : "MPU6050 calibration failed.");
  }

  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  initializeEscOutputs();
  writeAllMotorsMinimum();

  if (ENABLE_CAMERA) {
    cameraHealthy = initializeCamera();
    if (cameraHealthy) {
      Serial.println("Camera initialized.");
    } else {
      Serial.println("WARNING: Camera initialization failed; flight core remains available.");
    }
  }

  setupNetworkServices();

  xTaskCreatePinnedToCore(flightTask, "Flight250Hz", 8192, nullptr, 3,
                          &flightTaskHandle, 0);
  if (ENABLE_CAMERA && cameraHealthy) {
    xTaskCreatePinnedToCore(cameraTask, "CameraAnalysis", 8192, nullptr, 1,
                            &cameraTaskHandle, 1);
  }
  xTaskCreatePinnedToCore(serviceTask, "CommandTelemetry", 6144, nullptr, 1,
                          &serviceTaskHandle, 1);

  printHelp();
}

void loop() {
  // FreeRTOS tasks own all runtime work.
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ============================== FLIGHT TASK ==================================

void flightTask(void *parameter) {
  (void)parameter;
  uint32_t nextWakeUs = micros();

  for (;;) {
    nextWakeUs += CONTROL_PERIOD_US;

    // Runtime calibration is performed only in this task, preventing a second
    // task from using Wire concurrently with the 250 Hz sensor reader.
    if (calibrationRequested && !motorsArmed) {
      calibrationRequested = false;
      imuCalibrated = false;
      imuCalibrated = calibrateMPU();
      Serial.println(imuCalibrated ? "Calibration complete." : "Calibration failed.");
      nextWakeUs = micros();
    }

    float ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      imuHealthy = true;
      updateAttitude(ax, ay, az, gx, gy, gz);
      updateVibrationMonitor(ax, ay, az);
    } else {
      imuHealthy = false;
      disarmMotors("IMU read failure");
    }

    parseGPS();
    updateMissionLogic();
    runFlightController();

    const int32_t remainingUs = (int32_t)(nextWakeUs - micros());
    if (remainingUs > 1000) {
      vTaskDelay(pdMS_TO_TICKS((remainingUs - 500) / 1000));
    }
    while ((int32_t)(nextWakeUs - micros()) > 0) {
      taskYIELD();
    }

    // If computation overruns badly, restart timing instead of chasing backlog.
    if ((int32_t)(micros() - nextWakeUs) > (int32_t)CONTROL_PERIOD_US) {
      nextWakeUs = micros();
    }
  }
}

// =============================== MPU6050 =====================================

bool initializeMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75); // WHO_AM_I
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 1, true) != 1) return false;
  const uint8_t who = Wire.read();
  if (who != 0x68 && who != 0x69) return false;

  // Wake, select gyro PLL clock, configure DLPF and default full-scale ranges.
  const uint8_t registers[][2] = {
    {0x6B, 0x01}, // PWR_MGMT_1
    {0x1A, 0x03}, // DLPF ~44 Hz accel / ~42 Hz gyro
    {0x19, 0x03}, // internal 1 kHz / (1+3) = 250 Hz
    {0x1B, 0x00}, // gyro +/-250 dps
    {0x1C, 0x00}  // accel +/-2 g
  };

  for (const auto &entry : registers) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(entry[0]);
    Wire.write(entry[1]);
    if (Wire.endTransmission() != 0) return false;
  }
  delay(100);
  return true;
}

bool calibrateMPU(uint16_t samples) {
  // readMPU() subtracts the existing bias. Clear it so repeated calibration
  // measures a new absolute bias instead of only the residual.
  gyroBiasX = gyroBiasY = gyroBiasZ = 0.0f;
  double sx = 0, sy = 0, sz = 0;
  uint16_t accepted = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    float ax, ay, az, gx, gy, gz;
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sx += gx;
      sy += gy;
      sz += gz;
      ++accepted;
    }
    delay(2);
  }
  if (accepted < samples * 0.95f) return false;
  gyroBiasX = sx / accepted;
  gyroBiasY = sy / accepted;
  gyroBiasZ = sz / accepted;
  return true;
}

bool readMPU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, true) != 14) return false;

  const int16_t rawAx = (Wire.read() << 8) | Wire.read();
  const int16_t rawAy = (Wire.read() << 8) | Wire.read();
  const int16_t rawAz = (Wire.read() << 8) | Wire.read();
  (void)((Wire.read() << 8) | Wire.read()); // temperature
  const int16_t rawGx = (Wire.read() << 8) | Wire.read();
  const int16_t rawGy = (Wire.read() << 8) | Wire.read();
  const int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = ACCEL_X_SIGN * rawAx / 16384.0f;
  ay = ACCEL_Y_SIGN * rawAy / 16384.0f;
  az = ACCEL_Z_SIGN * rawAz / 16384.0f;
  gx = GYRO_X_SIGN * rawGx / 131.0f - gyroBiasX;
  gy = GYRO_Y_SIGN * rawGy / 131.0f - gyroBiasY;
  gz = GYRO_Z_SIGN * rawGz / 131.0f - gyroBiasZ;
  return true;
}

void updateAttitude(float ax, float ay, float az,
                    float gx, float gy, float gz) {
  static bool initialized = false;
  static float roll = 0, pitch = 0, yaw = 0;

  const float accelNorm = sqrtf(ax * ax + ay * ay + az * az);
  const bool accelTrustworthy = accelNorm > 0.75f && accelNorm < 1.25f;
  const float rollAcc = atan2f(ay, az) * RAD_TO_DEG;
  const float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  if (!initialized) {
    roll = rollAcc;
    pitch = pitchAcc;
    yaw = 0.0f;
    initialized = true;
  } else {
    roll += gx * CONTROL_DT;
    pitch += gy * CONTROL_DT;
    yaw = wrap360(yaw + gz * CONTROL_DT);
    if (accelTrustworthy) {
      roll = COMPLEMENTARY_ALPHA * roll + (1.0f - COMPLEMENTARY_ALPHA) * rollAcc;
      pitch = COMPLEMENTARY_ALPHA * pitch + (1.0f - COMPLEMENTARY_ALPHA) * pitchAcc;
    }
  }

  if (yawResetRequested) {
    portENTER_CRITICAL(&stateMux);
    yaw = wrap360(requestedYawDeg);
    yawResetRequested = false;
    portEXIT_CRITICAL(&stateMux);
  }

  GPSData localGps;
  portENTER_CRITICAL(&stateMux);
  localGps = gps;
  portEXIT_CRITICAL(&stateMux);

  // GPS course is meaningful only while translating. Fuse it very slowly.
  if (localGps.courseValid && localGps.speedMps >= GPS_COURSE_MIN_MPS) {
    const float courseError = wrap180(localGps.courseDeg - yaw);
    yaw = wrap360(yaw + 0.02f * courseError);
    headingReferenceSet = true;
  }

  portENTER_CRITICAL(&stateMux);
  attitude.rollDeg = roll;
  attitude.pitchDeg = pitch;
  attitude.yawDeg = yaw;
  attitude.gxDps = gx;
  attitude.gyDps = gy;
  attitude.gzDps = gz;
  portEXIT_CRITICAL(&stateMux);
}

void updateVibrationMonitor(float ax, float ay, float az) {
  // Separate slow airframe motion/gravity from fast vibration, then maintain an
  // exponentially weighted RMS. Thresholds require airframe-specific testing.
  static bool ready = false;
  static float slowAx = 0, slowAy = 0, slowAz = 1;
  static float meanSquare = 0;
  static uint16_t highCount = 0;

  if (!ready) {
    slowAx = ax; slowAy = ay; slowAz = az;
    ready = true;
  }
  constexpr float slowAlpha = 0.02f;
  slowAx += slowAlpha * (ax - slowAx);
  slowAy += slowAlpha * (ay - slowAy);
  slowAz += slowAlpha * (az - slowAz);
  const float hx = ax - slowAx;
  const float hy = ay - slowAy;
  const float hz = az - slowAz;
  const float instantSquare = hx * hx + hy * hy + hz * hz;
  meanSquare = 0.98f * meanSquare + 0.02f * instantSquare;
  vibrationRmsG = sqrtf(meanSquare);

  if (!motorsArmed || commandedThrottleUs < 1200) {
    highCount = 0;
    vibrationAlert = false;
    return;
  }

  if (vibrationRmsG > 0.35f) {
    if (highCount < 1000) ++highCount;
  } else if (vibrationRmsG < 0.20f) {
    highCount = 0;
    vibrationAlert = false;
  }

  if (highCount >= 250 && !vibrationAlert) { // approximately one second
    vibrationAlert = true;
    Serial.printf("VIBRATION ALERT: high-frequency RMS %.3f g\n", vibrationRmsG);
    if (ENABLE_VIBRATION_RTH && homeLocked) mission = MissionState::RETURN_HOME;
  }
}

// =============================== ESC OUTPUT =================================

static uint32_t pulseToDuty(uint16_t pulseUs) {
  const uint32_t maxDuty = (1UL << ESC_PWM_BITS) - 1UL;
  const uint32_t periodUs = 1000000UL / ESC_PWM_HZ;
  return ((uint32_t)pulseUs * maxDuty) / periodUs;
}

void initializeEscOutputs() {
  const int pins[4] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};
  for (uint8_t i = 0; i < 4; ++i) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pins[i], ESC_PWM_HZ, ESC_PWM_BITS, MOTOR_CHANNEL[i]);
#else
    ledcSetup(MOTOR_CHANNEL[i], ESC_PWM_HZ, ESC_PWM_BITS);
    ledcAttachPin(pins[i], MOTOR_CHANNEL[i]);
#endif
    writeEscMicroseconds(i, ESC_MIN_US);
  }
  delay(3000); // allow standard PWM ESCs to recognize minimum throttle
}

void writeEscMicroseconds(uint8_t motor, uint16_t pulseUs) {
  if (motor >= 4) return;
  pulseUs = constrain(pulseUs, ESC_MIN_US, ESC_MAX_US);
  const uint32_t duty = pulseToDuty(pulseUs);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(MOTOR_CHANNEL[motor], duty);
#else
  ledcWrite(MOTOR_CHANNEL[motor], duty);
#endif
}

void writeAllMotorsMinimum() {
  for (uint8_t i = 0; i < 4; ++i) writeEscMicroseconds(i, ESC_MIN_US);
}

void resetControllers() {
  AttitudeData a;
  portENTER_CRITICAL(&stateMux);
  a = attitude;
  portEXIT_CRITICAL(&stateMux);
  rollRatePID.reset(a.gxDps);
  pitchRatePID.reset(a.gyDps);
  yawRatePID.reset(a.gzDps);
}

bool armMotors() {
  if (BENCH_TEST_MODE) {
    Serial.println("BENCH mode: logical arming allowed, physical outputs remain at minimum.");
  }
  if (digitalRead(MISSION_SWITCH_PIN) != LOW) {
    Serial.println("ARM REFUSED: mission/arm switch must be LOW.");
    return false;
  }
  if (!imuHealthy || !imuCalibrated) {
    Serial.println("ARM REFUSED: IMU is not healthy and calibrated.");
    return false;
  }
  AttitudeData a;
  portENTER_CRITICAL(&stateMux);
  a = attitude;
  portEXIT_CRITICAL(&stateMux);
  if (fabsf(a.rollDeg) > 10.0f || fabsf(a.pitchDeg) > 10.0f) {
    Serial.println("ARM REFUSED: aircraft must be level within 10 degrees.");
    return false;
  }
  commandedThrottleUs = ESC_MIN_US;
  resetControllers();
  motorsArmed = true;
  mission = MissionState::STABILIZE;
  Serial.println("LOGIC ARMED. Raise THR gradually only during a controlled test.");
  return true;
}

void disarmMotors(const char *reason) {
  const bool wasArmed = motorsArmed;
  motorsArmed = false;
  mission = MissionState::DISARMED;
  commandedThrottleUs = ESC_MIN_US;
  writeAllMotorsMinimum();
  resetControllers();
  if (wasArmed && reason) Serial.printf("DISARMED: %s\n", reason);
}

void computeMotorMix(float throttleUs, float rollCorrection,
                     float pitchCorrection, float yawCorrection) {
  // Positive roll correction raises left motors. Positive pitch correction
  // raises rear motors. Positive yaw correction raises CCW rotors (M1/M3).
  float correction[4];
  correction[0] =  rollCorrection - pitchCorrection + yawCorrection;
  correction[1] = -rollCorrection - pitchCorrection - yawCorrection;
  correction[2] = -rollCorrection + pitchCorrection + yawCorrection;
  correction[3] =  rollCorrection + pitchCorrection - yawCorrection;

  float highest = correction[0], lowest = correction[0];
  for (uint8_t i = 1; i < 4; ++i) {
    highest = max(highest, correction[i]);
    lowest = min(lowest, correction[i]);
  }

  // If the requested differential is wider than the ESC's usable range,
  // scale all corrections together so roll/pitch/yaw proportions are retained.
  const float availableRange = ESC_MAX_US - ESC_IDLE_US;
  const float correctionSpan = highest - lowest;
  if (correctionSpan > availableRange) {
    const float scale = availableRange / correctionSpan;
    for (float &value : correction) value *= scale;
    highest *= scale;
    lowest *= scale;
  }

  // Choose the closest feasible common throttle while preserving differential.
  const float minimumBase = ESC_IDLE_US - lowest;
  const float maximumBase = ESC_MAX_US - highest;
  const float feasibleThrottle = constrain(throttleUs, minimumBase, maximumBase);
  for (uint8_t i = 0; i < 4; ++i) {
    computedMotorUs[i] = (uint16_t)constrain(feasibleThrottle + correction[i],
                                                    (float)ESC_IDLE_US,
                                                    (float)ESC_MAX_US);
  }
}

void runFlightController() {
  if (!motorsArmed || mission == MissionState::DISARMED ||
      mission == MissionState::EMERGENCY_STOP) {
    writeAllMotorsMinimum();
    return;
  }

  // The physical switch is a hard, continuously checked permission input.
  if (digitalRead(MISSION_SWITCH_PIN) != LOW) {
    disarmMotors("physical arm switch opened");
    return;
  }

  AttitudeData a;
  portENTER_CRITICAL(&stateMux);
  a = attitude;
  portEXIT_CRITICAL(&stateMux);

  if (fabsf(a.rollDeg) > EMERGENCY_TILT_DEG ||
      fabsf(a.pitchDeg) > EMERGENCY_TILT_DEG) {
    mission = MissionState::EMERGENCY_STOP;
    disarmMotors("extreme tilt detected");
    return;
  }

  const float rollRateSetpoint = constrain(
      rollAngleKp * (desiredRollDeg - a.rollDeg),
      -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);
  const float pitchRateSetpoint = constrain(
      pitchAngleKp * (desiredPitchDeg - a.pitchDeg),
      -MAX_ANGLE_RATE_DPS, MAX_ANGLE_RATE_DPS);

  const float rollCorrection = rollRatePID.update(
      rollRateSetpoint, a.gxDps, CONTROL_DT);
  const float pitchCorrection = pitchRatePID.update(
      pitchRateSetpoint, a.gyDps, CONTROL_DT);
  const float yawCorrection = yawRatePID.update(
      desiredYawRateDps, a.gzDps, CONTROL_DT);

  const float throttle = constrain(commandedThrottleUs,
                                   (float)ESC_MIN_US,
                                   (float)THROTTLE_MAX_US);
  if (throttle < ESC_IDLE_US) {
    writeAllMotorsMinimum();
    resetControllers();
    return;
  }

  computeMotorMix(throttle, rollCorrection, pitchCorrection, yawCorrection);
  for (uint8_t i = 0; i < 4; ++i) {
    writeEscMicroseconds(i, BENCH_TEST_MODE ? ESC_MIN_US : computedMotorUs[i]);
  }
}

// ================================= GPS =======================================

void parseGPS() {
  static char line[128];
  static uint8_t length = 0;

  while (Serial2.available()) {
    const char c = (char)Serial2.read();
    if (c == '\n') {
      line[length] = '\0';
      if (length > 6) parseNMEALine(line);
      length = 0;
    } else if (c != '\r') {
      if (length < sizeof(line) - 1) line[length++] = c;
      else length = 0; // discard overlong/corrupt sentence
    }
  }
}

bool validateNMEAChecksum(const char *line) {
  if (!line || line[0] != '$') return false;
  const char *star = strchr(line, '*');
  if (!star || strlen(star) < 3) return false;
  uint8_t checksum = 0;
  for (const char *p = line + 1; p < star; ++p) checksum ^= (uint8_t)*p;
  const uint8_t expected = (uint8_t)strtoul(star + 1, nullptr, 16);
  return checksum == expected;
}

double nmeaCoordinateToDegrees(const char *field, bool longitude) {
  if (!field || !*field) return 0.0;
  const uint8_t degreeDigits = longitude ? 3 : 2;
  char degreePart[4] = {0};
  strncpy(degreePart, field, degreeDigits);
  const double degrees = atof(degreePart);
  const double minutes = atof(field + degreeDigits);
  return degrees + minutes / 60.0;
}

bool parseNMEALine(char *line) {
  if (!validateNMEAChecksum(line)) return false;
  char copy[128];
  strncpy(copy, line, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = '\0';
  char *star = strchr(copy, '*');
  if (star) *star = '\0';

  // Preserve empty NMEA fields. strtok_r() would silently remove them and
  // shift every subsequent field number.
  char *fields[16] = {nullptr};
  uint8_t count = 1;
  fields[0] = copy;
  for (char *p = copy; *p && count < 16; ++p) {
    if (*p == ',') {
      *p = '\0';
      fields[count++] = p + 1;
    }
  }
  if (count < 2) return false;

  GPSData updated;
  portENTER_CRITICAL(&stateMux);
  updated = gps;
  portEXIT_CRITICAL(&stateMux);

  if (!strcmp(fields[0], "$GPGGA") || !strcmp(fields[0], "$GNGGA")) {
    if (count < 8) return false;
    const int fixQuality = atoi(fields[6]);
    updated.fix = fixQuality > 0;
    updated.satellites = (uint8_t)atoi(fields[7]);
    if (updated.fix && fields[2] && fields[3] && fields[4] && fields[5]) {
      updated.lat = nmeaCoordinateToDegrees(fields[2], false);
      updated.lng = nmeaCoordinateToDegrees(fields[4], true);
      if (fields[3][0] == 'S') updated.lat = -updated.lat;
      if (fields[5][0] == 'W') updated.lng = -updated.lng;
      updated.lastFixMs = millis();
    }
  } else if (!strcmp(fields[0], "$GPRMC") || !strcmp(fields[0], "$GNRMC")) {
    if (count < 9) return false;
    if (fields[2] && fields[2][0] == 'A') {
      updated.fix = true;
      updated.lat = nmeaCoordinateToDegrees(fields[3], false);
      updated.lng = nmeaCoordinateToDegrees(fields[5], true);
      if (fields[4][0] == 'S') updated.lat = -updated.lat;
      if (fields[6][0] == 'W') updated.lng = -updated.lng;
      updated.speedMps = atof(fields[7]) * 0.514444f; // knots -> m/s
      updated.courseDeg = wrap360(atof(fields[8]));
      updated.courseValid = updated.speedMps >= GPS_COURSE_MIN_MPS;
      updated.lastFixMs = millis();
    }
  } else {
    return false;
  }

  portENTER_CRITICAL(&stateMux);
  gps = updated;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  constexpr double earthRadiusM = 6371000.0;
  const double p1 = lat1 * DEG_TO_RAD;
  const double p2 = lat2 * DEG_TO_RAD;
  const double dp = (lat2 - lat1) * DEG_TO_RAD;
  const double dl = (lon2 - lon1) * DEG_TO_RAD;
  const double a = sin(dp / 2) * sin(dp / 2) +
                   cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
  return earthRadiusM * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

float bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
  const double p1 = lat1 * DEG_TO_RAD;
  const double p2 = lat2 * DEG_TO_RAD;
  const double dl = (lon2 - lon1) * DEG_TO_RAD;
  const double y = sin(dl) * cos(p2);
  const double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
  return wrap360(atan2(y, x) * RAD_TO_DEG);
}

float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float wrap360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

// ============================= MISSION LOGIC =================================

void updateMissionLogic() {
  GPSData g;
  portENTER_CRITICAL(&stateMux);
  g = gps;
  portEXIT_CRITICAL(&stateMux);

  if (!homeLocked && g.fix && g.satellites >= 6 &&
      millis() - g.lastFixMs < GPS_TIMEOUT_MS) {
    portENTER_CRITICAL(&stateMux);
    homeLat = g.lat;
    homeLng = g.lng;
    homeLocked = true;
    portEXIT_CRITICAL(&stateMux);
    Serial.printf("HOME LOCKED: %.7f, %.7f\n", g.lat, g.lng);
  }

  if (motorsArmed &&
      (mission == MissionState::FLY_TO_WAYPOINT ||
       mission == MissionState::RETURN_HOME ||
       mission == MissionState::SURVEY_GRID) &&
      (!g.fix || millis() - g.lastFixMs > GPS_TIMEOUT_MS)) {
    mission = MissionState::GPS_FAILSAFE_HOLD;
    desiredRollDeg = 0;
    desiredPitchDeg = 0;
    desiredYawRateDps = 0;
  }

  if (obstacleBrakeRequested &&
      (mission == MissionState::FLY_TO_WAYPOINT ||
       mission == MissionState::SURVEY_GRID)) {
    mission = MissionState::OBSTACLE_BRAKE;
  }

  updateNavigationSetpoints();
}

void updateNavigationSetpoints() {
  desiredRollDeg = 0.0f;
  desiredPitchDeg = 0.0f;
  desiredYawRateDps = 0.0f;

  if (mission == MissionState::STABILIZE ||
      mission == MissionState::OBSTACLE_BRAKE ||
      mission == MissionState::GPS_FAILSAFE_HOLD) {
    return;
  }

  if (mission == MissionState::RETURN_HOME) {
    double hLat, hLng;
    bool hasHome;
    portENTER_CRITICAL(&stateMux);
    hasHome = homeLocked;
    hLat = homeLat;
    hLng = homeLng;
    portEXIT_CRITICAL(&stateMux);
    if (!hasHome) {
      mission = MissionState::GPS_FAILSAFE_HOLD;
      return;
    }
    portENTER_CRITICAL(&stateMux);
    targetLat = hLat;
    targetLng = hLng;
    targetValid = true;
    portEXIT_CRITICAL(&stateMux);
  }

  if (mission != MissionState::FLY_TO_WAYPOINT &&
      mission != MissionState::RETURN_HOME &&
      mission != MissionState::SURVEY_GRID) return;

  GPSData g;
  AttitudeData a;
  portENTER_CRITICAL(&stateMux);
  g = gps;
  a = attitude;
  portEXIT_CRITICAL(&stateMux);

  double localTargetLat, localTargetLng;
  bool hasTarget;
  portENTER_CRITICAL(&stateMux);
  hasTarget = targetValid;
  localTargetLat = targetLat;
  localTargetLng = targetLng;
  portEXIT_CRITICAL(&stateMux);
  if (!g.fix || !hasTarget || !headingReferenceSet) return;

  const double distance = distanceMeters(g.lat, g.lng,
                                         localTargetLat, localTargetLng);
  const float bearing = bearingDegrees(g.lat, g.lng,
                                       localTargetLat, localTargetLng);
  const float headingError = wrap180(bearing - a.yawDeg);

  desiredYawRateDps = constrain(headingError * 1.3f, -70.0f, 70.0f);
  const float alignment = max(0.0f, cosf(headingError * DEG_TO_RAD));
  desiredPitchDeg = NAV_FORWARD_PITCH_DEG * alignment *
                    constrain((float)(distance / 25.0), 0.25f, 1.0f);
  // Small course-error bank aids the turn; yaw remains the primary command.
  desiredRollDeg = constrain(headingError * 0.08f,
                             -NAV_MAX_ROLL_DEG, NAV_MAX_ROLL_DEG);

  if (distance <= WAYPOINT_RADIUS_M) {
    if (mission == MissionState::SURVEY_GRID) {
      advanceSurveyWaypoint();
    } else {
      mission = MissionState::STABILIZE;
      desiredRollDeg = desiredPitchDeg = desiredYawRateDps = 0;
      Serial.println("Waypoint reached; switching to level stabilization.");
    }
  }
}

bool configureSurvey(double originLat, double originLng,
                     float sideKm, float spacingKm) {
  if (sideKm <= 0 || sideKm > 50.0f || spacingKm <= 0 || spacingKm > sideKm)
    return false;

  SurveyPlan newPlan;
  newPlan.originLat = originLat;
  newPlan.originLng = originLng;
  newPlan.sideKm = sideKm;
  newPlan.spacingKm = spacingKm;
  newPlan.pointsPerLine = (uint16_t)ceilf(sideKm / spacingKm) + 1;
  newPlan.lineCount = newPlan.pointsPerLine;
  newPlan.waypointCount = (uint32_t)newPlan.pointsPerLine * newPlan.lineCount;
  newPlan.waypointIndex = 0;
  newPlan.valid = true;

  portENTER_CRITICAL(&stateMux);
  survey = newPlan;
  targetLat = originLat;
  targetLng = originLng;
  targetValid = true;
  portEXIT_CRITICAL(&stateMux);
  return true;
}

bool surveyWaypoint(uint32_t index, double &lat, double &lng) {
  SurveyPlan plan;
  portENTER_CRITICAL(&stateMux);
  plan = survey;
  portEXIT_CRITICAL(&stateMux);
  if (!plan.valid || index >= plan.waypointCount) return false;
  const uint32_t line = index / plan.pointsPerLine;
  const uint32_t position = index % plan.pointsPerLine;
  const float northKm = min(line * plan.spacingKm, plan.sideKm);
  float eastKm = min(position * plan.spacingKm, plan.sideKm);
  if (line & 1U) eastKm = plan.sideKm - eastKm;

  lat = plan.originLat + northKm / 111.32;
  const double cosLat = cos(plan.originLat * DEG_TO_RAD);
  if (fabs(cosLat) < 0.01) return false;
  lng = plan.originLng + eastKm / (111.32 * cosLat);
  return true;
}

void advanceSurveyWaypoint() {
  uint32_t nextIndex, count;
  portENTER_CRITICAL(&stateMux);
  nextIndex = ++survey.waypointIndex;
  count = survey.waypointCount;
  if (nextIndex >= count) survey.valid = false;
  portEXIT_CRITICAL(&stateMux);

  if (nextIndex >= count) {
    mission = MissionState::RETURN_HOME;
    Serial.println("Survey complete; selecting return-home target.");
    return;
  }
  double nextLat, nextLng;
  if (!surveyWaypoint(nextIndex, nextLat, nextLng)) {
    mission = MissionState::GPS_FAILSAFE_HOLD;
    return;
  }
  portENTER_CRITICAL(&stateMux);
  targetLat = nextLat;
  targetLng = nextLng;
  targetValid = true;
  portEXIT_CRITICAL(&stateMux);
  Serial.printf("Survey waypoint %lu/%lu: %.7f, %.7f\n",
                (unsigned long)(nextIndex + 1),
                (unsigned long)count,
                nextLat, nextLng);
}

// ============================== CAMERA TASK ==================================

bool initializeCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_D0;
  config.pin_d1 = CAM_D1;
  config.pin_d2 = CAM_D2;
  config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4;
  config.pin_d5 = CAM_D5;
  config.pin_d6 = CAM_D6;
  config.pin_d7 = CAM_D7;
  config.pin_xclk = CAM_XCLK;
  config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href = CAM_HREF;
  config.pin_sccb_sda = I2C_SDA_PIN;
  config.pin_sccb_scl = I2C_SCL_PIN;
  config.pin_pwdn = -1;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA; // 160 x 120
  config.jpeg_quality = 15;
  config.fb_count = 1;
#if defined(CAMERA_GRAB_WHEN_EMPTY)
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
#endif
#if defined(CAMERA_FB_IN_DRAM)
  config.fb_location = CAMERA_FB_IN_DRAM;
#endif
  esp_err_t error = esp_camera_init(&config);
  if (error == ESP_OK) return true;

  // OV7670 is documented primarily with YUV/RGB output. Some driver versions
  // accept direct grayscale and others do not. Retry in YUV422 and extract Y.
  esp_camera_deinit();
  delay(50);
  config.pixel_format = PIXFORMAT_YUV422;
  error = esp_camera_init(&config);
  if (error == ESP_OK) {
    Serial.println("Camera grayscale unavailable; using YUV422 luminance fallback.");
    return true;
  }
  return false;
}

void cameraTask(void *parameter) {
  (void)parameter;
  for (;;) {
    camera_fb_t *frame = esp_camera_fb_get();
    if (frame &&
        (frame->format == PIXFORMAT_GRAYSCALE ||
         frame->format == PIXFORMAT_YUV422) &&
        frame->width == 160 && frame->height == 120) {
      analyzeCameraFrame(frame);
    }
    if (frame) esp_camera_fb_return(frame);
    vTaskDelay(pdMS_TO_TICKS(80)); // ~12.5 analysis frames/s
  }
}

uint8_t frameLuminance(const camera_fb_t *frame, int pixelIndex) {
  if (!frame || pixelIndex < 0) return 0;
  if (frame->format == PIXFORMAT_GRAYSCALE) {
    if ((size_t)pixelIndex >= frame->len) return 0;
    return frame->buf[pixelIndex];
  }
  // The esp32-camera YUV422 buffer uses two bytes per pixel with luma in the
  // even byte of each pixel pair representation used by the driver.
  const size_t byteIndex = (size_t)pixelIndex * 2U;
  if (byteIndex >= frame->len) return 0;
  return frame->buf[byteIndex];
}

void analyzeCameraFrame(camera_fb_t *frame) {
  // Lightweight background-difference candidate detector. It deliberately
  // does not claim identity, range, or thermal detection.
  static uint8_t background[160 * 120];
  static bool backgroundReady = false;
  static uint8_t candidatePersistence = 0;
  static uint32_t lastReportMs = 0;
  static long previousDarkCount = 0;
  static int previousCenterX = 80;
  static int previousCenterY = 60;

  if (!backgroundReady) {
    for (int i = 0; i < 160 * 120; ++i) {
      background[i] = frameLuminance(frame, i);
    }
    backgroundReady = true;
    return;
  }

  int minX = 159, maxX = 0, minY = 119, maxY = 0;
  uint32_t changed = 0;
  uint32_t darkCount = 0;
  uint32_t sumDarkX = 0, sumDarkY = 0;

  for (int y = 0; y < 120; y += 2) {
    for (int x = 0; x < 160; x += 2) {
      const int index = y * 160 + x;
      const uint8_t pixel = frameLuminance(frame, index);
      const uint8_t bg = background[index];
      if (abs((int)pixel - (int)bg) > 35) {
        ++changed;
        minX = min(minX, x); maxX = max(maxX, x);
        minY = min(minY, y); maxY = max(maxY, y);
      }
      if (pixel < 55) {
        ++darkCount;
        sumDarkX += x;
        sumDarkY += y;
      }
      // Slow IIR background adaptation: 31/32 old + 1/32 new.
      background[index] = (uint8_t)(((uint16_t)bg * 31U + pixel) >> 5);
    }
  }

  bool candidate = false;
  if (changed >= 70 && changed <= 1400 && maxX > minX && maxY > minY) {
    const float width = maxX - minX + 1;
    const float height = maxY - minY + 1;
    const float aspect = height / width;
    const float sampledBoxArea = (width * height) / 4.0f;
    const float fill = changed / max(1.0f, sampledBoxArea);
    candidate = aspect >= 1.15f && aspect <= 4.5f &&
                fill >= 0.12f && fill <= 0.90f;
  }

  candidatePersistence = candidate ? min(20, candidatePersistence + 1)
                                   : max(0, candidatePersistence - 1);
  if (candidatePersistence >= 6 && millis() - lastReportMs > 15000) {
    visualCandidateConfirmed = true;
    reportVisualCandidate();
    lastReportMs = millis();
    candidatePersistence = 0;
  }

  if (ENABLE_GESTURE_COMMANDS && darkCount > 300) {
    const int centerX = sumDarkX / darkCount;
    const int centerY = sumDarkY / darkCount;
    const int dx = centerX - previousCenterX;
    const int dy = centerY - previousCenterY;
    const long dArea = (long)darkCount - previousDarkCount;

    if (darkCount > 850 && abs(dx) < 8 && abs(dy) < 8) {
      obstacleBrakeRequested = true;
    } else if (dx < -20) {
      obstacleBrakeRequested = false;
      if (mission == MissionState::OBSTACLE_BRAKE)
        mission = MissionState::STABILIZE;
    } else if (dx > 20) {
      if (homeLocked) mission = MissionState::RETURN_HOME;
    }
    (void)dArea; // retained for future near/far gesture validation
    previousCenterX = centerX;
    previousCenterY = centerY;
    previousDarkCount = darkCount;
  } else if (darkCount <= 300) {
    previousDarkCount = 0;
  }
}

void reportVisualCandidate() {
  GPSData g;
  portENTER_CRITICAL(&stateMux);
  g = gps;
  portEXIT_CRITICAL(&stateMux);
  Serial.println("VISUAL CANDIDATE: persistent person-like moving region detected.");
  if (g.fix) {
    Serial.printf("Drone GPS at observation: %.7f, %.7f (not survivor range-fix)\n",
                  g.lat, g.lng);
    if (ENABLE_UDP_TELEMETRY && udpServiceReady &&
        WiFi.status() == WL_CONNECTED) {
      char alert[160];
      snprintf(alert, sizeof(alert),
               "ALERT=VISUAL_CANDIDATE,LAT=%.7f,LON=%.7f,"
               "COORDINATE_TYPE=DRONE_OBSERVATION",
               g.lat, g.lng);
      telemetryUdp.beginPacket(BASE_IP, TELEMETRY_PORT);
      telemetryUdp.print(alert);
      telemetryUdp.endPacket();
    }
  } else {
    Serial.println("No current GPS fix available for this observation.");
  }
  sendTelemetry();
}

// ========================== COMMANDS / TELEMETRY ============================

void serviceTask(void *parameter) {
  (void)parameter;
  uint32_t lastTelemetryMs = 0;
  for (;;) {
    processSerialCommands();
    if (WiFi.status() == WL_CONNECTED) {
      if (ENABLE_WIFI_OTA && !otaServiceReady) {
        ArduinoOTA.begin();
        otaServiceReady = true;
        Serial.println("OTA service ready while disarmed.");
      }
      if (ENABLE_UDP_TELEMETRY && !udpServiceReady) {
        telemetryUdp.begin(TELEMETRY_PORT);
        udpServiceReady = true;
      }
      // Do not even service an update request while motors are logically armed.
      if (ENABLE_WIFI_OTA && otaServiceReady && !motorsArmed) ArduinoOTA.handle();
    }
    if (millis() - lastTelemetryMs >= 1000) {
      sendTelemetry();
      lastTelemetryMs = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setupNetworkServices() {
  if (!ENABLE_WIFI_OTA && !ENABLE_UDP_TELEMETRY) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (ENABLE_WIFI_OTA) {
    ArduinoOTA.setHostname("sar-esp32-drone");
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]() {
      disarmMotors("OTA update requested");
      Serial.println("OTA update starting; motors locked.");
    });
    ArduinoOTA.onEnd([]() { Serial.println("OTA update complete."); });
  }
}

const char *missionName(MissionState state) {
  switch (state) {
    case MissionState::DISARMED: return "DISARMED";
    case MissionState::STABILIZE: return "STABILIZE";
    case MissionState::FLY_TO_WAYPOINT: return "WAYPOINT";
    case MissionState::RETURN_HOME: return "RTH";
    case MissionState::OBSTACLE_BRAKE: return "BRAKE";
    case MissionState::SURVEY_GRID: return "SURVEY";
    case MissionState::GPS_FAILSAFE_HOLD: return "GPS_FAILSAFE";
    case MissionState::EMERGENCY_STOP: return "EMERGENCY";
  }
  return "UNKNOWN";
}

void sendTelemetry() {
  GPSData g;
  AttitudeData a;
  portENTER_CRITICAL(&stateMux);
  g = gps;
  a = attitude;
  portEXIT_CRITICAL(&stateMux);

  char packet[300];
  snprintf(packet, sizeof(packet),
           "STATE=%s,ARM=%u,BENCH=%u,R=%.2f,P=%.2f,Y=%.2f,"
           "GPS=%u,SAT=%u,LAT=%.7f,LON=%.7f,SPD=%.2f,THR=%.0f,"
           "M=%u/%u/%u/%u,CANDIDATE=%u,VIB_RMS=%.3f,VIB_ALERT=%u",
           missionName(mission), motorsArmed, BENCH_TEST_MODE,
           a.rollDeg, a.pitchDeg, a.yawDeg,
           g.fix, g.satellites, g.lat, g.lng, g.speedMps,
           commandedThrottleUs,
           computedMotorUs[0], computedMotorUs[1],
           computedMotorUs[2], computedMotorUs[3],
           visualCandidateConfirmed, vibrationRmsG, vibrationAlert);

  if (ENABLE_UDP_TELEMETRY && udpServiceReady &&
      WiFi.status() == WL_CONNECTED) {
    telemetryUdp.beginPacket(BASE_IP, TELEMETRY_PORT);
    telemetryUdp.print(packet);
    telemetryUdp.endPacket();
  }
}

void printStatus() {
  GPSData g;
  AttitudeData a;
  double hLat, hLng;
  bool hasHome;
  portENTER_CRITICAL(&stateMux);
  g = gps;
  a = attitude;
  hLat = homeLat;
  hLng = homeLng;
  hasHome = homeLocked;
  portEXIT_CRITICAL(&stateMux);

  Serial.println("---------------- STATUS ----------------");
  Serial.printf("State: %s | logical arm: %s | bench lock: %s\n",
                missionName(mission), motorsArmed ? "YES" : "NO",
                BENCH_TEST_MODE ? "ON" : "OFF");
  Serial.printf("IMU: healthy=%u calibrated=%u | R/P/Y: %.2f %.2f %.2f deg\n",
                imuHealthy, imuCalibrated, a.rollDeg, a.pitchDeg, a.yawDeg);
  Serial.printf("Rates: %.2f %.2f %.2f dps | desired: %.2f %.2f yawRate %.2f\n",
                a.gxDps, a.gyDps, a.gzDps,
                desiredRollDeg, desiredPitchDeg, desiredYawRateDps);
  Serial.printf("GPS: fix=%u satellites=%u lat=%.7f lng=%.7f speed=%.2f m/s\n",
                g.fix, g.satellites, g.lat, g.lng, g.speedMps);
  Serial.printf("Home: %s %.7f %.7f | Heading reference: %s\n",
                hasHome ? "LOCKED" : "NOT LOCKED", hLat, hLng,
                headingReferenceSet ? "SET" : "NOT SET");
  Serial.printf("Throttle: %.0f | computed motors: %u %u %u %u\n",
                commandedThrottleUs, computedMotorUs[0], computedMotorUs[1],
                computedMotorUs[2], computedMotorUs[3]);
  Serial.printf("Vibration RMS: %.3f g | alert: %s | automatic RTH: %s\n",
                vibrationRmsG, vibrationAlert ? "YES" : "NO",
                ENABLE_VIBRATION_RTH ? "ENABLED" : "DISABLED");
  Serial.println("----------------------------------------");
}

void printHelp() {
  Serial.println("Commands (newline terminated):");
  Serial.println("  STATUS");
  Serial.println("  ARM 7319       - requires physical switch LOW; bench lock remains active");
  Serial.println("  DISARM");
  Serial.println("  CALIBRATE      - disarmed, level and completely still");
  Serial.println("  THR 1000..1750 - open-loop throttle; there is no altitude hold");
  Serial.println("  YAW degrees    - manually establish north-referenced yaw");
  Serial.println("  WP lat lon     - set a waypoint");
  Serial.println("  STARTWP        - start waypoint mission after logical arm");
  Serial.println("  SURVEY lat lon side_km spacing_km");
  Serial.println("  STARTSURVEY");
  Serial.println("  RTH | BRAKE | RESUME");
  Serial.println("  PIDR kp ki kd | PIDP kp ki kd | PIDY kp ki kd");
  Serial.println("  HELP");
}

void processSerialCommands() {
  static char buffer[128];
  static uint8_t length = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (length == 0) continue;
      buffer[length] = '\0';
      length = 0;

      if (!strcmp(buffer, "STATUS")) {
        printStatus();
      } else if (!strcmp(buffer, "HELP")) {
        printHelp();
      } else if (!strcmp(buffer, "DISARM")) {
        disarmMotors("operator command");
      } else if (!strcmp(buffer, "ARM 7319")) {
        armMotors();
      } else if (!strcmp(buffer, "CALIBRATE")) {
        if (motorsArmed) Serial.println("Calibration refused while armed.");
        else {
          calibrationRequested = true;
          Serial.println("Calibration requested; keep aircraft level and still.");
        }
      } else if (!strcmp(buffer, "RTH")) {
        if (motorsArmed && homeLocked) mission = MissionState::RETURN_HOME;
        else Serial.println("RTH refused: logical arm and home lock required.");
      } else if (!strcmp(buffer, "BRAKE")) {
        obstacleBrakeRequested = true;
        mission = MissionState::OBSTACLE_BRAKE;
      } else if (!strcmp(buffer, "RESUME")) {
        obstacleBrakeRequested = false;
        mission = motorsArmed ? MissionState::STABILIZE : MissionState::DISARMED;
      } else if (!strcmp(buffer, "STARTWP")) {
        GPSData g;
        portENTER_CRITICAL(&stateMux); g = gps; portEXIT_CRITICAL(&stateMux);
        if (motorsArmed && targetValid && g.fix && headingReferenceSet)
          mission = MissionState::FLY_TO_WAYPOINT;
        else
          Serial.println("STARTWP refused: arm, target, GPS and heading reference required.");
      } else if (!strcmp(buffer, "STARTSURVEY")) {
        GPSData g;
        portENTER_CRITICAL(&stateMux); g = gps; portEXIT_CRITICAL(&stateMux);
        if (motorsArmed && survey.valid && g.fix && headingReferenceSet)
          mission = MissionState::SURVEY_GRID;
        else
          Serial.println("STARTSURVEY refused: arm, survey, GPS and heading reference required.");
      } else {
        float value;
        double lat, lng;
        float sideKm, spacingKm;
        float kp, ki, kd;
        if (sscanf(buffer, "THR %f", &value) == 1) {
          commandedThrottleUs = constrain(value, (float)ESC_MIN_US,
                                          (float)THROTTLE_MAX_US);
          Serial.printf("Open-loop throttle set to %.0f us.\n", commandedThrottleUs);
        } else if (sscanf(buffer, "YAW %f", &value) == 1) {
          if (motorsArmed) {
            Serial.println("YAW reset refused while armed.");
          } else {
            portENTER_CRITICAL(&stateMux);
            requestedYawDeg = wrap360(value);
            yawResetRequested = true;
            portEXIT_CRITICAL(&stateMux);
            headingReferenceSet = true;
            Serial.printf("Yaw reference set to %.1f degrees.\n", wrap360(value));
          }
        } else if (sscanf(buffer, "WP %lf %lf", &lat, &lng) == 2) {
          if (lat >= -90 && lat <= 90 && lng >= -180 && lng <= 180) {
            portENTER_CRITICAL(&stateMux);
            targetLat = lat;
            targetLng = lng;
            targetValid = true;
            portEXIT_CRITICAL(&stateMux);
            Serial.printf("Waypoint set: %.7f, %.7f\n", lat, lng);
          } else Serial.println("Invalid waypoint coordinates.");
        } else if (sscanf(buffer, "SURVEY %lf %lf %f %f",
                          &lat, &lng, &sideKm, &spacingKm) == 4) {
          if (motorsArmed) {
            Serial.println("Survey reconfiguration refused while armed.");
          } else if (configureSurvey(lat, lng, sideKm, spacingKm))
            Serial.printf("Survey configured: %lu waypoints.\n",
                          (unsigned long)survey.waypointCount);
          else Serial.println("Invalid survey parameters.");
        } else if (sscanf(buffer, "PIDR %f %f %f", &kp, &ki, &kd) == 3) {
          if (motorsArmed || kp < 0 || ki < 0 || kd < 0)
            Serial.println("PID update refused: disarm and use non-negative gains.");
          else {
            rollRatePID.kp = kp; rollRatePID.ki = ki; rollRatePID.kd = kd;
            rollRatePID.reset(); Serial.println("Roll-rate PID updated.");
          }
        } else if (sscanf(buffer, "PIDP %f %f %f", &kp, &ki, &kd) == 3) {
          if (motorsArmed || kp < 0 || ki < 0 || kd < 0)
            Serial.println("PID update refused: disarm and use non-negative gains.");
          else {
            pitchRatePID.kp = kp; pitchRatePID.ki = ki; pitchRatePID.kd = kd;
            pitchRatePID.reset(); Serial.println("Pitch-rate PID updated.");
          }
        } else if (sscanf(buffer, "PIDY %f %f %f", &kp, &ki, &kd) == 3) {
          if (motorsArmed || kp < 0 || ki < 0 || kd < 0)
            Serial.println("PID update refused: disarm and use non-negative gains.");
          else {
            yawRatePID.kp = kp; yawRatePID.ki = ki; yawRatePID.kd = kd;
            yawRatePID.reset(); Serial.println("Yaw-rate PID updated.");
          }
        } else {
          Serial.println("Unknown command. Enter HELP.");
        }
      }
    } else if (length < sizeof(buffer) - 1) {
      buffer[length++] = (char)toupper((unsigned char)c);
    } else {
      length = 0;
    }
  }
}

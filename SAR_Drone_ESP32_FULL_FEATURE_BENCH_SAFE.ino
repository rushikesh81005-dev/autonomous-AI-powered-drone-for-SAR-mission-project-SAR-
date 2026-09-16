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
#include <Adafruit_BMP280.h>
#include <Adafruit_MLX90640.h>

// ============================ TinyML / FOMO ================================
// The inference adapter uses an Edge Impulse-exported FOMO model. The model
// itself is NOT fabricated here: a trained .eim/TFLite model must be exported
// from your own SAR/person dataset and the generated Edge Impulse Arduino
// library placed beside this sketch. The controller still compiles without it
// and falls back to the legacy visual candidate detector.
#ifndef ENABLE_FOMO_TINYML
#define ENABLE_FOMO_TINYML 1
#endif
#ifndef FOMO_MIN_CONFIDENCE
#define FOMO_MIN_CONFIDENCE 0.65f
#endif
#ifndef FOMO_INPUT_WIDTH
#define FOMO_INPUT_WIDTH 96
#endif
#ifndef FOMO_INPUT_HEIGHT
#define FOMO_INPUT_HEIGHT 96
#endif

#if ENABLE_FOMO_TINYML && __has_include("edge-impulse-sdk/classifier/ei_run_classifier.h")
  #include "edge-impulse-sdk/classifier/ei_run_classifier.h"
  #define FOMO_LIBRARY_PRESENT 1
#else
  #define FOMO_LIBRARY_PRESENT 0
#endif


#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// ============================= BUILD SAFETY ================================

static constexpr bool BENCH_TEST_MODE          = true;
static constexpr bool ENABLE_CAMERA            = true;
static constexpr bool ENABLE_GESTURE_COMMANDS  = true;
static constexpr bool ENABLE_VIBRATION_RTH     = true;
static constexpr bool ENABLE_WIFI_OTA          = true;
static constexpr bool ENABLE_UDP_TELEMETRY     = true;
static constexpr bool ENABLE_BATTERY_MONITOR   = true;
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
/*static const char *WIFI_SSID     = "Your-PC-Hotspot-Name";
static const char *WIFI_PASSWORD = "Your-PC-Hotspot-Password";
static const char *OTA_PASSWORD  = "Choose-a-strong-OTA-password";
static constexpr bool ENABLE_WIFI_OTA = true;*/


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
  EMERGENCY_STOP,
  LANDING
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
volatile bool fomoHealthy = false;
volatile bool fomoPersonCandidate = false;
volatile float fomoConfidence = 0.0f;
volatile float fomoCenterX = 0.5f;
volatile float fomoCenterY = 0.5f;
volatile uint32_t fomoInferenceMs = 0;
volatile uint32_t fomoInferenceCount = 0;
static camera_fb_t *fomoFrameForInference = nullptr;
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

// ======================== FULL SAR FEATURE MODULE ==========================
// Bench-safe integration. Physical ESC output remains hard-locked by
// BENCH_TEST_MODE. These modules implement sensing, logging, alerting,
// navigation state and analysis without providing a bypass for the lock.

static constexpr int SIM800_RX_PIN = 12; // ESP32 RX <- SIM800 TX
static constexpr int SIM800_TX_PIN = 2;  // ESP32 TX -> SIM800 RX (boot strap: validate hardware)
static constexpr uint32_t SIM800_BAUD = 9600;
static constexpr char RESCUE_PHONE[] = "+910000000000"; // replace before use
static constexpr uint8_t SMS_QUEUE_SIZE = 8;
static constexpr uint16_t EVIDENCE_MAX = 64;
static constexpr uint16_t POLYGON_MAX = 256;
static constexpr uint32_t GPS_STALE_MS = 2500;
static constexpr uint32_t BATTERY_SAMPLE_MS = 500;
static constexpr uint32_t BARO_SAMPLE_MS = 100;
static constexpr uint32_t THERMAL_SAMPLE_MS = 500;
static constexpr float LOW_BATTERY_V = 10.8f;
static constexpr float CRITICAL_BATTERY_V = 10.2f;
static constexpr float BATTERY_DIVIDER_SCALE = 1.0f; // INA219 reports bus V directly
static constexpr float LANDING_STEP_US = 1.0f;
static constexpr float LANDING_MIN_THR_US = 1000.0f;
static constexpr float THERMAL_HOT_DELTA_C = 8.0f;
static constexpr uint16_t THERMAL_MIN_HOT_PIXELS_FULL = 6;
static constexpr uint32_t MULTIFRAME_WINDOW_MS = 2500;
static constexpr uint8_t MULTIFRAME_REQUIRED = 3;
static constexpr float DUPLICATE_RADIUS_M = 12.0f;
static constexpr uint32_t DUPLICATE_WINDOW_MS = 120000;
static constexpr float CAMERA_HFOV_DEG = 60.0f;
static constexpr float CAMERA_VFOV_DEG = 45.0f;

// Forward declarations used by the feature module.
double distanceMeters(double lat1, double lon1, double lat2, double lon2);
void disarmMotors(const char *reason);
void queueRescueAlert(double lat,double lng,float temp);
bool enqueueSMS(const char *msg);
Adafruit_BMP280 fullBmp;
Adafruit_MLX90640 fullMlx;
HardwareSerial sim800(1);

volatile bool fullBmpHealthy=false, fullThermalHealthy=false, batteryHealthy=false;
volatile bool lowBattery=false, criticalBattery=false;
volatile float relativeAltitudeM=0, verticalRateMps=0, batteryVoltage=0, batteryCurrentA=0;
volatile float thermalMaxC=-100, thermalAverageC=-100;
volatile uint16_t thermalHotPixels=0;
volatile bool thermalHotspot=false;
volatile bool multimodalConfirmed=false;
volatile bool duplicateSuppressed=false;
volatile bool gpsLostFailsafe=false;
volatile bool landingRequested=false;

struct EvidenceRecord {
  uint32_t id=0, timeMs=0;
  double droneLat=0, droneLng=0;
  double estimatedLat=0, estimatedLng=0;
  float altitudeM=0, rollDeg=0, pitchDeg=0, yawDeg=0;
  float pixelX=0, pixelY=0, temperatureC=0;
  bool visible=false, thermal=false, confirmed=false, duplicate=false;
};
EvidenceRecord evidence[EVIDENCE_MAX];
uint16_t evidenceCount=0;
uint32_t evidenceNextId=1;

struct GeoPoint { double lat=0, lng=0; uint32_t timeMs=0; };
GeoPoint polygonLog[POLYGON_MAX];
uint16_t polygonCount=0;


struct SmsItem { char text[300]; uint8_t retries=0; uint32_t nextTry=0; bool used=false; };
SmsItem smsQueue[SMS_QUEUE_SIZE];
uint8_t smsHead=0, smsTail=0;

struct DetectionTrack { bool active=false; double lat=0,lng=0; uint32_t lastMs=0; uint8_t confirmations=0; };
DetectionTrack detectionTrack;

float baroGroundPressurePa=0;
uint32_t lastBaroMs=0,lastThermalMs=0,lastBatteryMs=0,lastGpsStateMs=0,lastSmsServiceMs=0;
uint8_t gpsLossConsecutive=0;
PID altitudePID={35.0f,2.0f,8.0f,0,0,120};
float desiredAltitudeM=2.0f;
float altitudePidOutputUs=0.0f;
uint32_t lastPolygonAutoMs=0;
bool lowBatteryAlertSent=false, criticalBatteryAlertSent=false;

bool fullI2CWrite8(uint8_t addr,uint8_t reg,uint8_t value){ Wire.beginTransmission(addr); Wire.write(reg); Wire.write(value); return Wire.endTransmission()==0; }
uint16_t readINA219BusRaw(){
  const uint8_t addr=0x40; Wire.beginTransmission(addr); Wire.write(0x02);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom((int)addr,2,true)!=2) return 0;
  return ((uint16_t)Wire.read()<<8)|Wire.read();
}
void initFullSensors(){
  fullBmpHealthy = fullBmp.begin(0x76,&Wire) || fullBmp.begin(0x77,&Wire);
  if(fullBmpHealthy){ fullBmp.setSampling(Adafruit_BMP280::MODE_NORMAL,Adafruit_BMP280::SAMPLING_X2,Adafruit_BMP280::SAMPLING_X16,Adafruit_BMP280::FILTER_X16,Adafruit_BMP280::STANDBY_MS_63); baroGroundPressurePa=fullBmp.readPressure(); }
  fullThermalHealthy = (fullMlx.begin(0x33,&Wire)==0);
  if(fullThermalHealthy){ fullMlx.setMode(MLX90640_CHESS); fullMlx.setResolution(MLX90640_ADC_18BIT); fullMlx.setRefreshRate(MLX90640_2_HZ); }
  // INA219 is probed by reading the bus-voltage register. A zero reading means absent.
  const uint16_t raw=readINA219BusRaw(); batteryHealthy=(raw!=0);
  sim800.begin(SIM800_BAUD,SERIAL_8N1,SIM800_RX_PIN,SIM800_TX_PIN);
}

void updateFullBarometer(){
  if(!fullBmpHealthy || millis()-lastBaroMs<BARO_SAMPLE_MS) return; lastBaroMs=millis();
  const float p=fullBmp.readPressure(); if(p<1000 || baroGroundPressurePa<1000) return;
  const float alt=44330.0f*(1.0f-powf(p/baroGroundPressurePa,0.19029495f));
  static float prev=0; static bool have=false; if(have) verticalRateMps=(alt-prev)/(BARO_SAMPLE_MS/1000.0f); prev=alt; have=true; relativeAltitudeM=0.85f*relativeAltitudeM+0.15f*alt;
}
void updateFullThermal(){
  if(!fullThermalHealthy || millis()-lastThermalMs<THERMAL_SAMPLE_MS) return; lastThermalMs=millis();
  float frame[32*24]; if(fullMlx.getFrame(frame)!=0) return;
  float sum=0,mx=-100; uint16_t hot=0; for(float t:frame){sum+=t; if(t>mx)mx=t;} const float avg=sum/768.0f; for(float t:frame) if(t>=avg+THERMAL_HOT_DELTA_C) hot++;
  thermalAverageC=avg; thermalMaxC=mx; thermalHotPixels=hot; thermalHotspot=(hot>=THERMAL_MIN_HOT_PIXELS_FULL);
}
void updateFullBattery(){
  if(millis()-lastBatteryMs<BATTERY_SAMPLE_MS) return; lastBatteryMs=millis();
  const uint16_t raw=readINA219BusRaw(); if(raw==0){batteryHealthy=false;return;} batteryHealthy=true; batteryVoltage=(raw>>3)*0.004f*BATTERY_DIVIDER_SCALE;
  lowBattery=batteryVoltage>0 && batteryVoltage<=LOW_BATTERY_V; criticalBattery=batteryVoltage>0 && batteryVoltage<=CRITICAL_BATTERY_V;
  if(lowBattery && !lowBatteryAlertSent){ char m[220]; snprintf(m,sizeof(m),"SAR LOW BATTERY %.2fV",batteryVoltage); enqueueSMS(m); lowBatteryAlertSent=true; Serial.println(m); }
  if(!lowBattery) lowBatteryAlertSent=false;
  if(criticalBattery && !criticalBatteryAlertSent){ char m[220]; snprintf(m,sizeof(m),"SAR CRITICAL BATTERY %.2fV EMERGENCY",batteryVoltage); enqueueSMS(m); criticalBatteryAlertSent=true; Serial.println(m); }
  if(!criticalBattery) criticalBatteryAlertSent=false;
}

void addPolygonPoint(double lat,double lng){ if(polygonCount<POLYGON_MAX) polygonLog[polygonCount++]={lat,lng,millis()}; }
double polygonAreaM2(){ if(polygonCount<3)return 0; double lat0=polygonLog[0].lat*DEG_TO_RAD; double area=0; for(uint16_t i=0;i<polygonCount;i++){uint16_t j=(i+1)%polygonCount; double x1=(polygonLog[i].lng-polygonLog[0].lng)*DEG_TO_RAD*cos(lat0)*6371000.0; double y1=(polygonLog[i].lat-polygonLog[0].lat)*DEG_TO_RAD*6371000.0; double x2=(polygonLog[j].lng-polygonLog[0].lng)*DEG_TO_RAD*cos(lat0)*6371000.0; double y2=(polygonLog[j].lat-polygonLog[0].lat)*DEG_TO_RAD*6371000.0; area+=x1*y2-x2*y1;} return fabs(area)*0.5; }

void cameraTiltCorrect(float px,float py,float &northM,float &eastM){
  const float alt=max(0.2f,relativeAltitudeM); const float nx=tanf((px-0.5f)*CAMERA_HFOV_DEG*DEG_TO_RAD); const float ey=tanf((py-0.5f)*CAMERA_VFOV_DEG*DEG_TO_RAD); AttitudeData a; portENTER_CRITICAL(&stateMux);a=attitude;portEXIT_CRITICAL(&stateMux);
  eastM=alt*nx; northM=alt*ey; const float r=a.rollDeg*DEG_TO_RAD,p=a.pitchDeg*DEG_TO_RAD; const float ce=cosf(r),se=sinf(r),cp=cosf(p),sp=sinf(p); const float e=eastM*ce+northM*se*sp; const float n=northM*cp-eastM*se*sp; eastM=e; northM=n;
}
void offsetToGPS(double lat,double lng,float northM,float eastM,double &olat,double &olng){ const double R=6371000.0; olat=lat+(northM/R)*RAD_TO_DEG; olng=lng+(eastM/(R*cos(lat*DEG_TO_RAD)))*RAD_TO_DEG; }

bool isDuplicate(double lat,double lng){ for(uint16_t i=0;i<evidenceCount;i++){EvidenceRecord &e=evidence[i]; if(!e.confirmed)continue; if(millis()-e.timeMs>DUPLICATE_WINDOW_MS)continue; if(distanceMeters(lat,lng,e.estimatedLat,e.estimatedLng)<=DUPLICATE_RADIUS_M)return true;} return false; }
void recordEvidence(float px,float py,bool vis,bool therm,float temp){ GPSData g;AttitudeData a;portENTER_CRITICAL(&stateMux);g=gps;a=attitude;portEXIT_CRITICAL(&stateMux); if(!g.fix)return; float n,e;cameraTiltCorrect(px,py,n,e); double elat,elon;offsetToGPS(g.lat,g.lng,n,e); const bool dup=isDuplicate(elat,elon); duplicateSuppressed=dup; if(evidenceCount<EVIDENCE_MAX){EvidenceRecord &r=evidence[evidenceCount++];r.id=evidenceNextId++;r.timeMs=millis();r.droneLat=g.lat;r.droneLng=g.lng;r.estimatedLat=elat;r.estimatedLng=elon;r.altitudeM=relativeAltitudeM;r.rollDeg=a.rollDeg;r.pitchDeg=a.pitchDeg;r.yawDeg=a.yawDeg;r.pixelX=px;r.pixelY=py;r.temperatureC=temp;r.visible=vis;r.thermal=therm;r.confirmed=(vis&&therm);r.duplicate=dup;} if(!dup && vis&&therm) queueRescueAlert(elat,elon,temp); }

void processMultiframeDetection(bool visible,bool thermal,float px,float py,float temp){
  const uint32_t now=millis(); if(now-detectionTrack.lastMs>MULTIFRAME_WINDOW_MS)detectionTrack.confirmations=0;
  if(visible||thermal)detectionTrack.confirmations++; else detectionTrack.confirmations=0; detectionTrack.lastMs=now;
  if(detectionTrack.confirmations>=MULTIFRAME_REQUIRED){ multimodalConfirmed=visible&&thermal; visualCandidateConfirmed=visible; if(multimodalConfirmed)recordEvidence(px,py,true,true,temp); detectionTrack.confirmations=0; }
}

bool enqueueSMS(const char *msg){uint8_t next=(smsTail+1)%SMS_QUEUE_SIZE;if(next==smsHead)return false;strncpy(smsQueue[smsTail].text,msg,sizeof(smsQueue[smsTail].text)-1);smsQueue[smsTail].text[sizeof(smsQueue[smsTail].text)-1]=0;smsQueue[smsTail].retries=0;smsQueue[smsTail].nextTry=millis();smsQueue[smsTail].used=true;smsTail=next;return true;}
void queueRescueAlert(double lat,double lng,float temp){char msg[300];snprintf(msg,sizeof(msg),"SAR ALERT: candidate confirmed. GPS %.7f,%.7f Temp %.1fC Maps: https://maps.google.com/?q=%.7f,%.7f",lat,lng,temp,lat,lng);enqueueSMS(msg);}
bool sendSMSNow(const char *msg){ sim800.println("AT+CMGF=1");delay(200);sim800.print("AT+CMGS=\"");sim800.print(RESCUE_PHONE);sim800.println("\"");delay(300);sim800.print(msg);sim800.write(26);uint32_t t=millis();while(millis()-t<7000){if(sim800.available()){String r=sim800.readString();if(r.indexOf("+CMGS")>=0||r.indexOf("OK")>=0)return true;}}return false; }
void serviceSMS(){if(millis()-lastSmsServiceMs<1000)return;lastSmsServiceMs=millis();if(smsHead==smsTail)return;if(millis()<smsQueue[smsHead].nextTry)return;if(sendSMSNow(smsQueue[smsHead].text)){smsQueue[smsHead].used=false;smsHead=(smsHead+1)%SMS_QUEUE_SIZE;}else{smsQueue[smsHead].retries++;if(smsQueue[smsHead].retries>=3){smsQueue[smsHead].used=false;smsHead=(smsHead+1)%SMS_QUEUE_SIZE;}else smsQueue[smsHead].nextTry=millis()+5000;}}


void updateAltitudePIDFramework(){
  const float err=desiredAltitudeM-relativeAltitudeM;
  altitudePidOutputUs=altitudePID.update(desiredAltitudeM,relativeAltitudeM,0.1f);
}
void filterGPSData(){
  static bool init=false; static double fl=0,fn=0; GPSData g;portENTER_CRITICAL(&stateMux);g=gps;portEXIT_CRITICAL(&stateMux); if(!g.fix)return; if(!init){fl=g.lat;fn=g.lng;init=true;} else {fl=0.8*fl+0.2*g.lat;fn=0.8*fn+0.2*g.lng;} portENTER_CRITICAL(&stateMux);gps.lat=fl;gps.lng=fn;portEXIT_CRITICAL(&stateMux);
}
void autoLogPolygon(){
  if(mission!=MissionState::SURVEY_GRID || millis()-lastPolygonAutoMs<2000)return; lastPolygonAutoMs=millis(); GPSData g;portENTER_CRITICAL(&stateMux);g=gps;portEXIT_CRITICAL(&stateMux);if(g.fix)addPolygonPoint(g.lat,g.lng);
}

void updateFullSafety(){
  filterGPSData(); updateAltitudePIDFramework(); autoLogPolygon();
  GPSData g;portENTER_CRITICAL(&stateMux);g=gps;portEXIT_CRITICAL(&stateMux); const bool stale=!g.fix || millis()-g.lastFixMs>GPS_STALE_MS; gpsLostFailsafe=stale;
  if(motorsArmed && stale && (mission==MissionState::SURVEY_GRID||mission==MissionState::FLY_TO_WAYPOINT||mission==MissionState::RETURN_HOME)){mission=MissionState::GPS_FAILSAFE_HOLD;}
  if(motorsArmed && criticalBattery){mission=MissionState::EMERGENCY_STOP;disarmMotors("critical battery");}
  if(motorsArmed && lowBattery && homeLocked && mission!=MissionState::RETURN_HOME){mission=MissionState::RETURN_HOME;}
  AttitudeData a;portENTER_CRITICAL(&stateMux);a=attitude;portEXIT_CRITICAL(&stateMux); if(fabsf(a.rollDeg)>EMERGENCY_TILT_DEG||fabsf(a.pitchDeg)>EMERGENCY_TILT_DEG){mission=MissionState::EMERGENCY_STOP;disarmMotors("excessive tilt");}
}
void updateLandingState(){
  if(mission!=MissionState::LANDING)return; desiredRollDeg=0;desiredPitchDeg=0;desiredYawRateDps=0;
  // Bench-safe landing profile: state machine only; physical ESCs remain locked.
  if(commandedThrottleUs>LANDING_MIN_THR_US)commandedThrottleUs=max(LANDING_MIN_THR_US,commandedThrottleUs-LANDING_STEP_US);
  if(relativeAltitudeM<=0.20f){commandedThrottleUs=ESC_MIN_US;disarmMotors("landing state complete");}
}
void exportPolygon(){Serial.printf("POLYGON_COUNT=%u AREA_M2=%.2f\n",polygonCount,polygonAreaM2());for(uint16_t i=0;i<polygonCount;i++)Serial.printf("P,%u,%.7f,%.7f,%lu\n",i,polygonLog[i].lat,polygonLog[i].lng,(unsigned long)polygonLog[i].timeMs);}
void printEvidence(){Serial.printf("EVIDENCE_COUNT=%u\n",evidenceCount);for(uint16_t i=0;i<evidenceCount;i++){EvidenceRecord&e=evidence[i];Serial.printf("E,%lu,%.7f,%.7f,%.7f,%.7f,A%.1f,T%.1f,C%u,D%u\n",(unsigned long)e.id,e.droneLat,e.droneLng,e.estimatedLat,e.estimatedLng,e.altitudeM,e.temperatureC,e.confirmed,e.duplicate);}}

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
void updateLandingState();
void updateFullBarometer();
void updateFullThermal();
void updateFullBattery();
void updateFullSafety();
void serviceSMS();
void addPolygonPoint(double lat,double lng);
void exportPolygon();
void printEvidence();
void processMultiframeDetection(bool visible,bool thermal,float px,float py,float temp);
bool enqueueSMS(const char *msg);
void queueRescueAlert(double lat,double lng,float temp);

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
  initFullSensors();
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
    updateFullBarometer();
    updateFullBattery();
    updateFullSafety();
    updateLandingState();
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

#if FOMO_LIBRARY_PRESENT
static int fomoSignalGetData(size_t offset, size_t length, float *out_ptr) {
  if (!fomoFrameForInference || !out_ptr) return -1;
  const int srcW = fomoFrameForInference->width;
  const int srcH = fomoFrameForInference->height;
  // The FOMO model is trained on grayscale 96x96 in the supplied integration
  // contract. Bilinear interpolation is intentionally avoided to keep latency
  // low; nearest-neighbour sampling is deterministic and ESP32 friendly.
  const size_t total = (size_t)FOMO_INPUT_WIDTH * FOMO_INPUT_HEIGHT;
  for (size_t i = 0; i < length; ++i) {
    const size_t idx = offset + i;
    if (idx >= total) { out_ptr[i] = 0; continue; }
    const int ox = (int)(idx % FOMO_INPUT_WIDTH);
    const int oy = (int)(idx / FOMO_INPUT_WIDTH);
    const int sx = min(srcW - 1, (ox * srcW) / FOMO_INPUT_WIDTH);
    const int sy = min(srcH - 1, (oy * srcH) / FOMO_INPUT_HEIGHT);
    out_ptr[i] = (float)frameLuminance(fomoFrameForInference, sy * srcW + sx);
  }
  return 0;
}

bool runFomoPersonDetector(camera_fb_t *frame, float &cx, float &cy, float &confidence) {
  if (!frame || frame->width != 160 || frame->height != 120) return false;
  const uint32_t t0 = millis();
  fomoFrameForInference = frame;
  signal_t signal;
  signal.total_length = (size_t)FOMO_INPUT_WIDTH * FOMO_INPUT_HEIGHT;
  signal.get_data = fomoSignalGetData;

  ei_impulse_result_t result = {};
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  fomoFrameForInference = nullptr;
  fomoInferenceMs = millis() - t0;
  ++fomoInferenceCount;
  if (err != EI_IMPULSE_OK) {
    fomoHealthy = false;
    return false;
  }
  fomoHealthy = true;

  bool person = false;
  float best = 0.0f;
  float bestCx = 0.5f, bestCy = 0.5f;
  // FOMO exports object detections as bounding_boxes. The object label used by
  // the training dataset must be exactly "person" (case-insensitive).
  for (size_t i = 0; i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; ++i) {
    const auto &bb = result.bounding_boxes[i];
    if (bb.value <= 0.0f || bb.label == nullptr) continue;
    bool isPerson = strcasecmp(bb.label, "person") == 0 ||
                    strcasecmp(bb.label, "human") == 0 ||
                    strcasecmp(bb.label, "survivor") == 0;
    if (isPerson && bb.value > best) {
      best = bb.value;
      bestCx = ((float)bb.x + 0.5f * (float)bb.width) / (float)FOMO_INPUT_WIDTH;
      bestCy = ((float)bb.y + 0.5f * (float)bb.height) / (float)FOMO_INPUT_HEIGHT;
      person = true;
    }
  }
  confidence = best;
  cx = constrain(bestCx, 0.0f, 1.0f);
  cy = constrain(bestCy, 0.0f, 1.0f);
  fomoConfidence = best;
  fomoPersonCandidate = person && best >= FOMO_MIN_CONFIDENCE;
  fomoCenterX = cx;
  fomoCenterY = cy;
  return fomoPersonCandidate;
}
#else
bool runFomoPersonDetector(camera_fb_t *frame, float &cx, float &cy, float &confidence) {
  (void)frame; cx = 0.5f; cy = 0.5f; confidence = 0.0f;
  fomoHealthy = false; fomoPersonCandidate = false;
  return false;
}
#endif

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

  float fomoCx = 0.5f, fomoCy = 0.5f, fomoConf = 0.0f;
  const bool fomoCandidate = ENABLE_FOMO_TINYML && runFomoPersonDetector(frame, fomoCx, fomoCy, fomoConf);

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

  const bool legacyVisibleCandidate = candidate || darkCount > 300;
  const bool visibleCandidate = fomoCandidate || legacyVisibleCandidate;
  // Prefer the neural-network centroid whenever FOMO produces a person.
  const float legacyPx = (darkCount > 0) ? ((float)(sumDarkX / max(1UL, darkCount)) / (float)frame->width) : 0.5f;
  const float legacyPy = (darkCount > 0) ? ((float)(sumDarkY / max(1UL, darkCount)) / (float)frame->height) : 0.5f;
  const float px = fomoCandidate ? fomoCx : legacyPx;
  const float py = fomoCandidate ? fomoCy : legacyPy;
  processMultiframeDetection(visibleCandidate, thermalHotspot, px, py, thermalMaxC);
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
    updateFullThermal();
    serviceSMS();
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
    case MissionState::LANDING: return "LANDING";
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
           "M=%u/%u/%u/%u,CANDIDATE=%u,VIB_RMS=%.3f,VIB_ALERT=%u,BARO_M=%.2f,BAT_V=%.2f,LOWBAT=%u,CRITBAT=%u,THERM_MAX=%.1f,HOTP=%u,CONF=%u,DUP=%u,GPSLOSS=%u,EVID=%u,POLY=%u,ALT_PID=%.1f",
           missionName(mission), motorsArmed, BENCH_TEST_MODE,
           a.rollDeg, a.pitchDeg, a.yawDeg,
           g.fix, g.satellites, g.lat, g.lng, g.speedMps,
           commandedThrottleUs,
           computedMotorUs[0], computedMotorUs[1],
           computedMotorUs[2], computedMotorUs[3],
           visualCandidateConfirmed, vibrationRmsG, vibrationAlert, relativeAltitudeM,batteryVoltage,lowBattery,criticalBattery,thermalMaxC,thermalHotPixels,multimodalConfirmed,duplicateSuppressed,gpsLostFailsafe,evidenceCount,polygonCount,altitudePidOutputUs);

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
  Serial.printf("Baro altitude: %.2f m | vertical rate: %.2f m/s | BMP=%u | AltPID=%.1f us\n",relativeAltitudeM,verticalRateMps,fullBmpHealthy,altitudePidOutputUs);
  Serial.printf("Battery: %.2f V healthy=%u low=%u critical=%u\n",batteryVoltage,batteryHealthy,lowBattery,criticalBattery);
  Serial.printf("Thermal: max=%.1fC avg=%.1fC hotPixels=%u hotspot=%u confirmed=%u\n",thermalMaxC,thermalAverageC,thermalHotPixels,thermalHotspot,multimodalConfirmed);
  Serial.printf("FOMO: library=%u healthy=%u person=%u conf=%.2f center=(%.2f,%.2f) infer_ms=%lu count=%lu\n", FOMO_LIBRARY_PRESENT, fomoHealthy, fomoPersonCandidate, fomoConfidence, fomoCenterX, fomoCenterY, (unsigned long)fomoInferenceMs, (unsigned long)fomoInferenceCount);
  Serial.printf("Evidence=%u Polygon=%u Area=%.2f m2 SMS=%u\n",evidenceCount,polygonCount,polygonAreaM2(),(smsTail+SMS_QUEUE_SIZE-smsHead)%SMS_QUEUE_SIZE);
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
  Serial.println("  RTH | BRAKE | RESUME | LAND");
  Serial.println("  POLY lat lon | POLYSHOW | EVIDENCE");
  Serial.println("  SMS text - queue rescue SMS");
  Serial.println("  PIDR kp ki kd | PIDP kp ki kd | PIDY kp ki kd | PIDA kp ki kd");
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
      } else if (!strcmp(buffer, "LAND")) {
        if(motorsArmed) { mission=MissionState::LANDING; landingRequested=true; Serial.println("Landing state requested (bench-safe; ESC lock remains active)."); } else Serial.println("LAND refused: not logically armed.");
      } else if (!strcmp(buffer, "POLYSHOW")) {
        exportPolygon();
      } else if (!strcmp(buffer, "EVIDENCE")) {
        printEvidence();
      } else if (!strncmp(buffer,"POLY ",5)) {
        double la,lo; if(sscanf(buffer,"POLY %lf %lf",&la,&lo)==2 && la>=-90&&la<=90&&lo>=-180&&lo<=180){addPolygonPoint(la,lo);Serial.printf("Polygon point %u added.\n",polygonCount);} else Serial.println("Invalid polygon point.");
      } else if (!strncmp(buffer,"SMS ",4)) {
        if(enqueueSMS(buffer+4)) Serial.println("SMS queued."); else Serial.println("SMS queue full.");
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
        } else if (sscanf(buffer, "PIDA %f %f %f", &kp, &ki, &kd) == 3) {
          if (motorsArmed || kp < 0 || ki < 0 || kd < 0) Serial.println("Altitude PID update refused: disarm and use non-negative gains.");
          else { altitudePID.kp=kp; altitudePID.ki=ki; altitudePID.kd=kd; altitudePID.reset(relativeAltitudeM); Serial.println("Altitude PID framework updated."); }
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

/*
// ============================================================================
// OPTIONAL ADD-ON CODE: BMP280 + MLX90640 + SX127x LORA
// ============================================================================
// This entire block is intentionally disabled. Remove only this opening and
// closing block-comment marker after the hardware is wired, pins are assigned,
// required libraries are installed, and bench testing is complete.
//
// Required Arduino libraries:
//   - Adafruit BMP280 Library
//   - Adafruit MLX90640
//   - RadioLib
//
// Safety: Do not enable autonomous takeoff or landing from a BMP280 alone.
// A downward range sensor, battery monitor, physical arm switch, manual
// emergency stop, and fully validated control loops are mandatory. This
// extension therefore leaves automatic takeoff/landing disabled.

#include <Adafruit_BMP280.h>
#include <Adafruit_MLX90640.h>
#include <RadioLib.h>

// ------------------------- FEATURE SWITCHES ---------------------------------
static constexpr bool ENABLE_BMP280_ALTITUDE     = false;
static constexpr bool ENABLE_MLX90640_THERMAL    = false;
static constexpr bool ENABLE_SX127X_FALLBACK     = false;
static constexpr bool ENABLE_TRACK_LOG           = false;
static constexpr bool ENABLE_AUTO_TAKEOFF_LAND   = false; // Must remain false

// ----------------------------- I2C SENSORS ----------------------------------
// BMP280 and MLX90640 share GPIO 21/22 with the MPU6050. Before enabling,
// protect every Wire access in readMPU() and these functions with one mutex.
static constexpr uint8_t BMP280_ADDR_PRIMARY     = 0x76;
static constexpr uint8_t BMP280_ADDR_ALTERNATE   = 0x77;
static constexpr uint8_t MLX90640_ADDR           = 0x33;
static constexpr uint32_t ADDON_SENSOR_PERIOD_MS = 500; // 2 Hz maximum
static constexpr float THERMAL_DELTA_C            = 8.0f;
static constexpr uint16_t THERMAL_MIN_HOT_PIXELS = 4;

Adafruit_BMP280 bmp280;
Adafruit_MLX90640 mlx90640;
SemaphoreHandle_t sharedI2cMutex = nullptr;

volatile bool bmp280Healthy = false;
volatile bool mlx90640Healthy = false;
volatile bool thermalCandidate = false;
volatile float baroAltitudeM = 0.0f;
float groundPressurePa = 0.0f;

// ------------------------------- SX127x -------------------------------------
// Replace every -1 with a physically wired, electrically validated GPIO.
// The current camera/ESC/GPS pin map does not provide a safe default SPI map.
// Do not use ESP32 boot-strapping pins 0, 2, or 12 without hardware validation.
static constexpr int LORA_SCK_PIN  = -1;
static constexpr int LORA_MISO_PIN = -1;
static constexpr int LORA_MOSI_PIN = -1;
static constexpr int LORA_NSS_PIN  = -1;
static constexpr int LORA_RST_PIN  = -1;
static constexpr int LORA_DIO0_PIN = -1;
static constexpr int LORA_DIO1_PIN = -1; // Optional; module dependent

// Set the frequency only to the band authorised for your radio and location.
static constexpr float LORA_FREQUENCY_MHZ = 0.0f;
static constexpr float LORA_BANDWIDTH_KHZ = 125.0f;
static constexpr uint8_t LORA_SPREADING_FACTOR = 9;
static constexpr uint8_t LORA_CODING_RATE = 7;
static constexpr int8_t LORA_TX_POWER_DBM = 10;

// Use SX1278 for common 433 MHz boards; use SX1276 for common 868/915 MHz
// boards. Select exactly one line after identifying the actual radio chip.
//#define USE_SX1278
#if defined(USE_SX1278)
SX1278 lora = new Module(LORA_NSS_PIN, LORA_DIO0_PIN, LORA_RST_PIN,
                         LORA_DIO1_PIN);
#else
SX1276 lora = new Module(LORA_NSS_PIN, LORA_DIO0_PIN, LORA_RST_PIN,
                         LORA_DIO1_PIN);
#endif

volatile bool loraHealthy = false;

// --------------------------- FLIGHT-TRACK LOG ------------------------------
struct TrackPoint {
  double lat;
  double lng;
  uint32_t timeMs;
};

static constexpr uint8_t TRACK_LOG_SIZE = 120;
TrackPoint trackLog[TRACK_LOG_SIZE];
uint8_t trackWriteIndex = 0;
uint8_t trackCount = 0;

bool lockSharedI2c(TickType_t waitTicks = pdMS_TO_TICKS(20)) {
  return sharedI2cMutex &&
         xSemaphoreTake(sharedI2cMutex, waitTicks) == pdTRUE;
}

void unlockSharedI2c() {
  if (sharedI2cMutex) xSemaphoreGive(sharedI2cMutex);
}

bool initializeAddOnHardware() {
  sharedI2cMutex = xSemaphoreCreateMutex();
  if (!sharedI2cMutex) return false;

  if (ENABLE_BMP280_ALTITUDE) {
    if (!lockSharedI2c()) return false;
    bmp280Healthy = bmp280.begin(BMP280_ADDR_PRIMARY, &Wire) ||
                    bmp280.begin(BMP280_ADDR_ALTERNATE, &Wire);
    if (bmp280Healthy) {
      groundPressurePa = bmp280.readPressure();
    }
    unlockSharedI2c();
  }

  if (ENABLE_MLX90640_THERMAL) {
    if (!lockSharedI2c()) return false;
    mlx90640Healthy = mlx90640.begin(MLX90640_ADDR, &Wire);
    if (mlx90640Healthy) {
      mlx90640.setMode(MLX90640_CHESS);
      mlx90640.setResolution(MLX90640_ADC_18BIT);
      mlx90640.setRefreshRate(MLX90640_2_HZ);
    }
    unlockSharedI2c();
  }

  if (ENABLE_SX127X_FALLBACK) {
    if (LORA_SCK_PIN < 0 || LORA_MISO_PIN < 0 || LORA_MOSI_PIN < 0 ||
        LORA_NSS_PIN < 0 || LORA_RST_PIN < 0 || LORA_DIO0_PIN < 0 ||
        LORA_FREQUENCY_MHZ <= 0.0f) {
      Serial.println("LoRa disabled: configure validated pins and frequency.");
      return false;
    }
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
    const int radioState = lora.begin(LORA_FREQUENCY_MHZ,
                                      LORA_BANDWIDTH_KHZ,
                                      LORA_SPREADING_FACTOR,
                                      LORA_CODING_RATE,
                                      0x12, LORA_TX_POWER_DBM);
    loraHealthy = (radioState == RADIOLIB_ERR_NONE);
  }
  return bmp280Healthy || mlx90640Healthy || loraHealthy;
}

void updateAddOnSensors() {
  static uint32_t lastRunMs = 0;
  if (millis() - lastRunMs < ADDON_SENSOR_PERIOD_MS) return;
  lastRunMs = millis();

  if (ENABLE_BMP280_ALTITUDE && bmp280Healthy && groundPressurePa > 0.0f &&
      lockSharedI2c()) {
    const float pressurePa = bmp280.readPressure();
    if (pressurePa > 1000.0f) {
      baroAltitudeM = 44330.0f *
          (1.0f - powf(pressurePa / groundPressurePa, 0.19029495f));
    }
    unlockSharedI2c();
  }

  if (ENABLE_MLX90640_THERMAL && mlx90640Healthy && lockSharedI2c()) {
    float thermalFrame[32 * 24];
    const int frameStatus = mlx90640.getFrame(thermalFrame);
    unlockSharedI2c();
    if (frameStatus != 0) return;

    float averageC = 0.0f;
    for (float pixelC : thermalFrame) averageC += pixelC;
    averageC /= (32.0f * 24.0f);

    uint16_t hotPixels = 0;
    for (float pixelC : thermalFrame) {
      if (pixelC >= averageC + THERMAL_DELTA_C) ++hotPixels;
    }
    thermalCandidate = hotPixels >= THERMAL_MIN_HOT_PIXELS;
  }
}

void recordTrackPoint() {
  if (!ENABLE_TRACK_LOG) return;
  GPSData localGps;
  portENTER_CRITICAL(&stateMux);
  localGps = gps;
  portEXIT_CRITICAL(&stateMux);
  if (!localGps.fix) return;

  trackLog[trackWriteIndex] = {localGps.lat, localGps.lng, millis()};
  trackWriteIndex = (trackWriteIndex + 1) % TRACK_LOG_SIZE;
  if (trackCount < TRACK_LOG_SIZE) ++trackCount;
}

bool sendLoRaFallbackTelemetry() {
  const bool wifiTelemetryAvailable = ENABLE_UDP_TELEMETRY &&
                                      udpServiceReady &&
                                      WiFi.status() == WL_CONNECTED;
  if (!ENABLE_SX127X_FALLBACK || !loraHealthy || wifiTelemetryAvailable)
    return false;

  GPSData localGps;
  AttitudeData localAttitude;
  portENTER_CRITICAL(&stateMux);
  localGps = gps;
  localAttitude = attitude;
  portEXIT_CRITICAL(&stateMux);

  char payload[190];
  snprintf(payload, sizeof(payload),
           "S=%s,G=%u,LA=%.6f,LO=%.6f,A=%.1f,T=%u,TH=%u,BA=%.1f",
           missionName(mission), localGps.fix, localGps.lat, localGps.lng,
           baroAltitudeM, thermalCandidate, loraHealthy, vibrationRmsG);
  return lora.transmit(payload) == RADIOLIB_ERR_NONE;
}

// Call updateAddOnSensors() and recordTrackPoint() from serviceTask() only
// after protecting every existing MPU6050 Wire transaction with sharedI2cMutex.
//
// Call initializeAddOnHardware() from setup() after Wire.begin().
//
// Call sendLoRaFallbackTelemetry() from sendTelemetry() after the Wi-Fi UDP
// attempt. LoRa is a low-bandwidth telemetry fallback; it is not a substitute
// for the physical arm switch, local serial control, or emergency disarm.
//
// Do not implement automatic takeoff or landing until a downward range sensor
// and a battery monitor are installed and validated. A BMP280 alone is not
// sufficient for reliable ground detection or a safe landing decision.
// ============================================================================
*/

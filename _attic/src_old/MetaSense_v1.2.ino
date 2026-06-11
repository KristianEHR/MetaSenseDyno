// MetaSense v1.2.4 (13.05.26)— KD V3.4 (06.04.26)
// ─── Board Selection ─────────────────────────────────────────────
// Set to 1 for ESP32-S3, 0 for ESP32 Dev Board
#define IS_ESP32S3 1  // CHANGE THIS BASED ON YOUR BOARD

#include <Arduino.h>
#include "MetaSense.h"//#include <ModbusTCPServer.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <deque>
#include <vector>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <sstream>
#include <cmath>
#include <Wire.h>
#include <Adafruit_MCP9600.h>
#include <Update.h>
#include <algorithm>
#include <utility>
#include <Adafruit_NAU7802.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h> // Library for AHT20
#include <cstring>
//patch 0
#include <Adafruit_Sensor.h>
#include <esp_adc_cal.h>
#include "driver/adc.h"
#include <algorithm>
#include <utility>
#include "driver/twai.h"
#include <math.h>
#include "Telemetry.h"
#include "LeafCan.h"
#include "DynoControl.h"
#include "Notify.h"
#include "globals.h"

// ─── Pin Configuration (Auto-selected based on board) ───────────
#if IS_ESP32S3
// ESP32-S3 pins
const int HALL_PIN = 26;
const int DRUM_PIN = 25;
const int I2C_SDA = 17;
const int I2C_SCL = 18;

#else
// ESP32 Dev Board pins
const int HALL_PIN = 4;
const int DRUM_PIN = 5;
const int I2C_SDA = 1;
const int I2C_SCL = 2;
#endif
//#include <esp_adc_cal.h>
// ============================================================
//  MetaSense Dyno Controller + Leaf ZE1 Inverter Feedback
//  ESP32-S3 TWAI CAN + Ramp + Feed-Forward + PI + Safety
// ============================================================


#define GEARING 11.0 / 16.0  //16/11 gear ratio
#define arm_length 0.2       //arm_length [m]
#define Torque_ratio 7.2727  // = 5 * GR = 5 * 1.4545 = 7.2727
#define Pi 3.1416
#define LED_PIN 48   // ESP32-S3 DevKitC-1 onboard RGB data pin
#define LED_COUNT 1  // just one pixel onboard
#define LED_TYPE NEO_GRB + NEO_KHZ800
#define SDA_PIN 17
#define SCL_PIN 18
#define BUS_FREQ 100000
// patch 0 end

// ─── Version Info ──esp32 or esp32-s3--───────────────────────────
#define FIRMWARE_VERSION "1.2.4"
#define BUILD_DATE "14.05.26"

// ============================================================
//  LEAF INVERTER FEEDBACK STRUCT
// ============================================================

struct LeafInvFeedback {
  float rpm = 0.0f;
  float torqueNm = 0.0f;

  float invTempC = 0.0f;
  float motorTempC = 0.0f;
  float coolantTempC = 0.0f;

  bool invReady = false;
  bool invFault = false;
  bool hvInterlockOk = false;
  uint8_t gearState = 0;
};

LeafInvFeedback leafFb;

// ============================================================
//  TWAI CAN INIT + SEND
// ============================================================

void canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
}

void canSend(uint32_t id, uint8_t dlc, const uint8_t *data) {
  twai_message_t msg = {};
  msg.identifier = id;
  msg.data_length_code = dlc;
  msg.extd = 0;
  msg.rtr = 0;

  for (uint8_t i = 0; i < dlc; ++i)
    msg.data[i] = data[i];

  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// ============================================================
//  LEAF ZE1 CAN DECODE
// ============================================================

static inline uint16_t u16_le(const uint8_t *d) {
  return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}

void decodeLeafMotorSpeed(const twai_message_t &msg) {
  uint16_t raw = u16_le(msg.data + 0);
  leafFb.rpm = (float)raw;  // 1 rpm/bit
}

void decodeLeafMotorTorque(const twai_message_t &msg) {
  int16_t raw = (int16_t)u16_le(msg.data + 0);
  leafFb.torqueNm = (float)raw * 0.5f;  // 0.5 Nm/bit
}

void decodeLeafTemps(const twai_message_t &msg) {
  leafFb.invTempC = (float)((int8_t)msg.data[0]) - 40.0f;
  leafFb.motorTempC = (float)((int8_t)msg.data[1]) - 40.0f;
  leafFb.coolantTempC = (float)((int8_t)msg.data[2]) - 40.0f;
}

void decodeLeafStatus(const twai_message_t &msg) {
  uint8_t b0 = msg.data[0];
  leafFb.invReady = (b0 & 0x01) != 0;
  leafFb.invFault = (b0 & 0x02) != 0;
  leafFb.hvInterlockOk = (b0 & 0x04) != 0;
  leafFb.gearState = msg.data[1];
}

void leafCanDecode(const twai_message_t &msg) {
  switch (msg.identifier) {
    case 0x1DA: decodeLeafMotorSpeed(msg); break;
    case 0x1DB: decodeLeafMotorTorque(msg); break;
    case 0x1DC: decodeLeafTemps(msg); break;
    case 0x1D4: decodeLeafStatus(msg); break;
    default: break;
  }
}
void updateTelemetryFromLeaf()
{
  MetaSense::live.leaf_rpm          = leafFb.rpm;
  MetaSense::live.leaf_torqueNm     = leafFb.torqueNm;
  MetaSense::live.leaf_invTempC     = leafFb.invTempC;
  MetaSense::live.leaf_motorTempC   = leafFb.motorTempC;
  MetaSense::live.leaf_coolantTempC = leafFb.coolantTempC;
  MetaSense::live.leaf_invReady     = leafFb.invReady;
  MetaSense::live.leaf_invFault     = leafFb.invFault;
  MetaSense::live.leaf_hvInterlockOk= leafFb.hvInterlockOk;
  MetaSense::live.leaf_gearState    = leafFb.gearState;
}

// ============================================================
//  CAN TORQUE OUTPUT + FAULT BROADCAST
// ============================================================
//
//bool isRecording = false;
const uint32_t CAN_ID_TORQUE_CMD = 0x200;
const uint32_t CAN_ID_FAULT_STAT = 0x201;

enum FaultCode : uint16_t {
  FAULT_NONE = 0,
  FAULT_ESTOP = 1,
  FAULT_OVERSPEED = 2,
  FAULT_OVERCURRENT = 3
};

FaultCode activeFaultCode = FAULT_NONE;

void sendCanTorque(float torqueNm) {
  if (torqueNm > 3276.7f) torqueNm = 3276.7f;
  if (torqueNm < -3276.8f) torqueNm = -3276.8f;

  int16_t raw = (int16_t)(torqueNm * 10.0f);

  uint8_t d[2];
  d[0] = raw & 0xFF;
  d[1] = raw >> 8;

  canSend(CAN_ID_TORQUE_CMD, 2, d);
}

void sendCanFaultStatus(uint8_t faultState) {
  uint8_t d[3];
  d[0] = faultState;
  d[1] = activeFaultCode & 0xFF;
  d[2] = activeFaultCode >> 8;

  canSend(CAN_ID_FAULT_STAT, 3, d);
}

// ============================================================
//  DYN0 CONTROL SYSTEM (Ramp + PI + FF + Safety + Slew Limit)
// ============================================================

const float dt_ctrl = 0.02f;
const float dt_inv = 0.10f;

const float J = 0.05f;

const float TORQUE_MIN = -40.0f;
const float TORQUE_MAX = 40.0f;

const float TORQUE_SLEW = 200.0f;
const float BRAKE_TORQUE_FAULT = -20.0f;

const float ADC_REF_MV = 3300.0f;

const float RPM_MAX = 18000.0f;
const float RPM_PER_MV = RPM_MAX / ADC_REF_MV;

const float IDC_MAX = 36.0f;
const float IDC_PER_MV = IDC_MAX / ADC_REF_MV;

const float RPM_LIMIT = RPM_MAX;
const float IDC_LIMIT = IDC_MAX;

enum FaultState {
  STATE_OK = 0,
  STATE_FAULT = 1
};

FaultState faultState = STATE_OK;

// ---------------------- RAMP ----------------------

struct BiExpRamp {
  float rpm;
  float target;
  float alphaUp;
  float alphaDown;

  void init(float startRpm) {
    rpm = startRpm;
    target = startRpm;
    alphaUp = 1.0f - expf(-dt_ctrl / 5.0f);
    alphaDown = 1.0f - expf(-dt_ctrl / 1.5f);
  }

  float update() {
    float alpha = (target > rpm) ? alphaUp : alphaDown;
    rpm += alpha * (target - rpm);
    return rpm;
  }
};

// ---------------------- PI ----------------------

struct PIController {
  float kp, ki;
  float integral;
  float tMin, tMax;

  void init(float kp_, float ki_, float tMin_, float tMax_) {
    kp = kp_;
    ki = ki_;
    tMin = tMin_;
    tMax = tMax_;
    integral = 0.0f;
  }

  float update(float setpoint, float measured) {
    float error = setpoint - measured;
    integral += error * ki * dt_ctrl;

    float out = kp * error + integral;

    if (out > tMax) {
      out = tMax;
      if (error > 0) integral -= error * ki * dt_ctrl;
    } else if (out < tMin) {
      out = tMin;
      if (error < 0) integral -= error * ki * dt_ctrl;
    }

    return out;
  }
};

// ---------------------- SAFETY ----------------------

bool checkFaultConditions(float rpmMeas, float idcMeas) {

//  if (digitalRead(PIN_ESTOP) == LOW) {
//    activeFaultCode = FAULT_ESTOP;
//    return true;
//  }
  if (rpmMeas > RPM_LIMIT) {
    activeFaultCode = FAULT_OVERSPEED;
    return true;
  }
  if (idcMeas > IDC_LIMIT) {
    activeFaultCode = FAULT_OVERCURRENT;
    return true;
  }

  activeFaultCode = FAULT_NONE;
  return false;
}

void updateFaultState(float rpmMeas, float idcMeas) {
  if (faultState == STATE_OK && checkFaultConditions(rpmMeas, idcMeas)) {
    faultState = STATE_FAULT;
    sendCanFaultStatus(STATE_FAULT);
  }
}

void resetFault() {
  if (faultState == STATE_FAULT) { //&& digitalRead(PIN_RESET) == HIGH) {
    faultState = STATE_OK;
    activeFaultCode = FAULT_NONE;
    sendCanFaultStatus(STATE_OK);
  }
}

float applySafetyGate(float requestedTorque) {
  if (faultState == STATE_FAULT)
    return BRAKE_TORQUE_FAULT;

  return requestedTorque;
}

// ============================================================
//  GLOBAL CONTROL VARIABLES
// ============================================================

BiExpRamp rpmRamp;
PIController speedPI;

float lastOmega = 0.0f;
float torqueHold = 0.0f;
unsigned long lastInvUpdateMs = 0;

// ============================================================
//  ADC HELPERS
// ============================================================

float readRpmFromAdcMv(float mv) {
  return mv * RPM_PER_MV;
}
float readIdcFromAdcMv(float mv) {
  return mv * IDC_PER_MV;
}


// --- ADC calibration storage ---
esp_adc_cal_characteristics_t adc1_chars;

// --- ADC channel selection ---
#define ADC_CH0 ADC1_CHANNEL_0  // GPIO1
#define ADC_CH1 ADC1_CHANNEL_1  // GPIO1
#define ADC_CH2 ADC1_CHANNEL_2  // GPIO1
#define ADC_CH3 ADC1_CHANNEL_3  // GPIO4
#define ADC_CH4 ADC1_CHANNEL_4  // GPIO5
uint32_t mv0 = 0;
uint32_t mv1 = 0;
uint32_t mv2 = 0;
uint32_t mv3 = 0;
uint32_t mv4 = 0;
// ─── AP Credentials ──────────────────────────────────────────────
const char *ap_ssid = "metasense";
const char *ap_password = "metasense";

// ─── Hall debounce (µs) ──────────────────────────────────────────
#define ENGINE_DEBOUNCE_US 200
#define DRUM_DEBOUNCE_US 30

// ─── Buffer Sizes ────────────────────────────────────────────────
const int BUFFER_SIZE = 12;
const int MEDIAN_SIZE = 5;
const int MAX_RUN_POINTS = 1500;
const int MAX_SAVED_RUNS = 10;
const int EGT_FILTER_SIZE = 5;

// ─── Update Rates (Hz) ───────────────────────────────────────────
const int SENSOR_UPDATE_MS = 20;  // 50Hz
const int WS_UPDATE_MS = 50;      // 20Hz max
const int EGT_UPDATE_MS = 66;     // 15Hz (fast enough for reliable reads)
const int GEAR_UPDATE_MS = 100;   // 10Hz
const int BMP_UPDATE_MS = 500;    // 2Hz

// ─── Safety Limits ──────────────────────────────────────────────
const float MAX_SAFE_DRUM_RPM = 14000.0f;
const float MIN_RPM_THRESHOLD_BRAKE = 35.0f;
const float setPoint_max = 2000.0f;  // rpm = 2000 rpm
const int OUTPUT_MIN = -1024;
const int OUTPUT_MAX = 512;

// ─── MCP9600 addresses ──────────────────────────────────────────
const uint8_t MCP_EGT_ADDR = 0x67;
Adafruit_MCP9600 mcp_egt;
bool mcp_egt_ok = false;
// patch 0
volatile float ExhaustTemp;
volatile float Ambiente;
volatile float h_reg[15];  //h_reg[0-7] are AD 0-7 inputs and h_reg[8-19] are variables transferred to modbus client
enum class State {
  Start,
  Motor,
  Idle,
  Dyno
};
int zz = 1, n = 0, j, sw = 0, sign = 0;
int rampup = 0;
int interval = 1000;  // interval at which to step rpm (milliseconds)
int VCU_ready = 0;
State prev_state = State::Start;
State state = State::Start;
const double k_reg[15] = { 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1e-3, 1.0, 1.0, 1.0, 1 };
static double setPoint = 0, setPoint_r = 0, b_Torq = 0, zero_offset = 0, rpm, tachogen, tachogen_r, prev_rpm;
static double KP = 0.06, KI = 0.30, KD = 0, Torque = 0, e_torque = 0;
//patch 0 end

// ─── EGT tracking with circular buffer ──────────────────────────
struct EGTTracker {
  float buffer[EGT_FILTER_SIZE];
  int index = 0;
  int count = 0;
  float filtered = 0.0f;
  float peak = 0.0f;
  float final = 0.0f;
  float ambient = 0.0f;

  void update(float raw, float amb) {
    buffer[index] = raw;
    index = (index + 1) % EGT_FILTER_SIZE;
    if (count < EGT_FILTER_SIZE) count++;

    float sum = 0;
    for (int i = 0; i < count; i++) sum += buffer[i];
    filtered = sum / count;

    if (filtered > peak) peak = filtered;
    final = filtered;
    if (amb > -50 && amb < 200) ambient = amb;
  }

  void reset() {
    peak = 0.0f;
    final = 0.0f;
    count = 0;
    index = 0;
    memset(buffer, 0, sizeof(buffer));
  }

  float getFiltered() {
    return filtered;
  }
  float getAmbient() {
    return ambient;
  }
};

EGTTracker egtTracker;

// ─── Optimized Circular Buffers (no std::deque) ─────────────────
struct SmoothingBuffer {
  float buffer[9];
  int index = 0;
  int count = 0;

  SmoothingBuffer() {
    memset(buffer, 0, sizeof(buffer));
  }

  float add(float value) {
    buffer[index] = value;
    index = (index + 1) % 9;
    if (count < 9) count++;

    float sum = 0;
    for (int i = 0; i < count; i++) sum += buffer[i];
    return sum / count;
  }

  void reset() {
    index = 0;
    count = 0;
    memset(buffer, 0, sizeof(buffer));
  }
};

// ─── Run history storage ────────────────────────────────────────
struct RunInfo {
  unsigned long timestamp;
  String filename;
  String customer;
  String vehicle;
  float peakHP;
  float peakTorque;
  float peakEGT;
};

std::vector<RunInfo> savedRuns;

// ─── Global variables ───────────────────────────────────────────
AsyncWebServer server(80);
//AsyncWebSocket ws("/ws");//mooved to globals.cpp
DNSServer dnsServer;

Adafruit_NAU7802 nau;
bool nau_initialized = false;

Adafruit_BMP280 bmp;
bool bmp_initialized = false;
//Patch 01

Adafruit_AHTX0 aht;
bool aht_initialized = false;

//NeoPixel 4_colour LED setup
Adafruit_NeoPixel strip(1, LED_PIN, LED_TYPE);

//WiFiClient wclient;

//IPAddress WAGO_server(192, 168, 0, 164);
//WiFiServer wifiserver(502);
//ModbusTCPServer modbusTCPServer;
//ModbusTCPClient modbusTCPClient(wclient);
//AutoPID myPID(&tachogen_r, &setPoint_r, &b_Torq, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
//end Patch 01

// Optimized median filters
float rpmMedian[MEDIAN_SIZE];
int rpmMedIdx = 0;
float drumMedian[MEDIAN_SIZE];
int drumMedIdx = 0;

float RPM_FILTER_ALPHA = 0.50f;

float CALIBRATION_FACTOR = 0.028;
float LEVER_ARM_METERS = 0.20;
float MAX_RPM = 16000;
float MAX_HP = 25;
float MAX_TORQUE = 25;
float PULSES_PER_REVOLUTION = 2.0f;
float PULSES_PER_REV_DRUM = 10.0f;
float BRAKE_TO_ENGINE_RATIO = 1.0f;
float DRUM_MASS = 10.0f;
float DRUM_DIAM_METERS = 0.3f;
float AUTO_HYSTERESIS = 0.0f;

String DYNO_MODE = "brake";
String DRUM_INERTIA_TYPE = "solid";
float DRUM_WALL_THICKNESS_CM = 2.0f;
float DRUM_INERTIA_CUSTOM = 0.5f;
String RPM_SOURCE = "spark";
float DRIVETRAIN_EFFICIENCY = 1.0f;
float VIRTUAL_GEAR_RATIO = 1.0f;
sensors_event_t humidity, temp_aht;
float ENERGY = 1.0f;
/*
// ─── Optimized Gear Ratio Tracker ────────────────────────────────
struct GearRatioTracker {
  float current_ratio = 11.0 / 16;
  float confidence = 0.0f;
  float filtered_ratio = 1.0f;
  int stable_samples = 0;
  float last_valid = 1.0f;
  unsigned long last_update = 0;
  float ratio_history[10] = { 0 };
  int history_index = 0;

  void update(float engine_rpm, float drum_rpm) {
    if (engine_rpm > 800.0f && drum_rpm > 20.0f) {
      float instant_ratio = engine_rpm / drum_rpm;

      if (instant_ratio > 0.5f && instant_ratio < 50.0f) {
        ratio_history[history_index] = instant_ratio;
        history_index = (history_index + 1) % 10;

        float sum = 0, sum_sq = 0;
        int valid_samples = 0;
        for (int i = 0; i < 10; i++) {
          if (ratio_history[i] > 0.5f) {
            sum += ratio_history[i];
            sum_sq += ratio_history[i] * ratio_history[i];
            valid_samples++;
          }
        }

        if (valid_samples >= 5) {
          float mean = sum / valid_samples;
          float variance = (sum_sq / valid_samples) - (mean * mean);
          float stability = 1.0f - constrain(variance / (mean * mean), 0.0f, 0.5f);

          if (stability > 0.98f && valid_samples >= 8) {
            stable_samples++;
            confidence = min(1.0f, confidence + 0.02f);
          } else if (stability > 0.95f) {
            stable_samples = max(0, stable_samples - 1);
            confidence = max(0.0f, confidence - 0.005f);
          } else {
            stable_samples = max(0, stable_samples - 2);
            confidence = max(0.0f, confidence - 0.01f);
          }

          float filter_strength = (confidence > 0.8f && stability > 0.98f) ? 0.01f : (confidence < 0.3f) ? 0.1f
                                                                                                         : 0.03f;

          filtered_ratio = filtered_ratio * (1.0f - filter_strength) + instant_ratio * filter_strength;

          if (confidence > 0.6f) {
            current_ratio = filtered_ratio;
            last_valid = filtered_ratio;
            last_update = millis();
          }
        }
      }
    } else if (confidence > 0) {
      confidence = max(0.0f, confidence - 0.002f);
      if (millis() - last_update > 5000) filtered_ratio *= 0.999f;
    }
  }

  void reset() {
    current_ratio = 1.0f;
    confidence = 0.0f;
    filtered_ratio = 1.0f;
    stable_samples = 0;
    last_valid = 1.0f;
    last_update = 0;
    memset(ratio_history, 0, sizeof(ratio_history));
    history_index = 0;
  }

  float getRatio() {
    return (confidence > 0.5f) ? current_ratio : last_valid;
  }
  float getConfidence() {
    return confidence;
  }
};

GearRatioTracker gearTracker;
/*
struct LiveData {
  float rpm = 0;
  float drum_rpm = 0;
  float hp = 0;
  float torque = 0;
  float peakHP = 0;
  float brakeTorque = 0;
  float peakHP_RPM = 0;
  float peakTorque = 0;
  float peakTorque_RPM = 0;
  float air_density = 1.204;
  float ambient_temp = 20.0;
  float pressure = 1013.25;
  float load_kg = 0;
};
LiveData current;
*/

float atmospheric_pressure = 1013.25;
float ambient_temp_c = 20.0;
float air_density = 1.204;
const float STD_AIR_DENSITY = 1.204;
//patch 02
float H2O_pp_correction_factor = 1.0;
//patch 02 end
volatile uint32_t drumPulseInterval = 0;
volatile uint32_t lastDrumPulseTime = 0;
volatile uint32_t lastPulseTime = 0;
volatile uint32_t pulseInterval = 0;

portMUX_TYPE rpmMux = portMUX_INITIALIZER_UNLOCKED;

// ISR handlers
void IRAM_ATTR hallISR() {
  uint32_t now = micros();
  uint32_t dt = now - lastPulseTime;
  if (dt < ENGINE_DEBOUNCE_US) return;
  //  portENTER_CRITICAL_ISR(&rpmMux);
  pulseInterval = dt;
  lastPulseTime = now;
  //  portEXIT_CRITICAL_ISR(&rpmMux);
}

void IRAM_ATTR drumISR() {
  uint32_t now = micros();
  uint32_t dt = now - lastDrumPulseTime;
  if (dt < DRUM_DEBOUNCE_US) return;
  //  portENTER_CRITICAL_ISR(&rpmMux);
  drumPulseInterval = dt;
  lastDrumPulseTime = now;
  //  portEXIT_CRITICAL_ISR(&rpmMux);
}

// Recording state
bool recording = false;
bool autoMode = false;
float autoStartRPM = 0;
float autoEndRPM = 0;

struct RunPoint {
  float rpm, hp, torque;
};
std::vector<RunPoint> currentRun;

float runPeakHP = 0, runPeakHP_RPM = 0;
float runPeakTorque = 0, runPeakTorque_RPM = 0;

bool settingsPageActive = false;
bool trendsPageActive = false;

Preferences preferences;

String sta_ssid = "";
String sta_password = "";
String currentCustomer = "";
String currentUnit = "";
String currentComments = "";

unsigned long recordingStartedMs = 0;

TaskHandle_t calibTask = NULL;
volatile float calibKnownWeight = 0.0f;
volatile bool calibRequest = false;

int32_t zeroOffset = 0;

SmoothingBuffer hpSmooth;
SmoothingBuffer torqueSmooth;

// Function prototypes
void broadcastSettings();
void sendRunComplete(float peakHP, float peakHP_RPM, float peakTorque, float peakTorque_RPM);
void saveFullRun();
void loadRunList();
void sendRunList();
void sendRunData(String filename);
void validatePins();

// ─── Fast median filter for 5 elements ──────────────────────────
float median5(float a, float b, float c, float d, float e) {
  if (a > b) std::swap(a, b);
  if (c > d) std::swap(c, d);
  if (a > c) std::swap(a, c);
  if (b > d) std::swap(b, d);
  if (e > b) std::swap(e, b);
  if (e > c) std::swap(e, c);
  if (a > e) std::swap(a, e);
  if (b > c) std::swap(b, c);
  return b;
}

// ─── Pin validation ──────────────────────────────────────────────
void validatePins() {
  Serial.println("\n🔧 Validating pin configuration...");
  if (digitalPinToInterrupt(HALL_PIN) == NOT_AN_INTERRUPT) {
    Serial.printf("⚠️  WARNING: Pin %d (HALL_PIN) doesn't support interrupts!\n", HALL_PIN);
  } else {
    Serial.printf("✅ Pin %d (HALL_PIN) supports interrupts\n", HALL_PIN);
  }
  if (digitalPinToInterrupt(DRUM_PIN) == NOT_AN_INTERRUPT) {
    Serial.printf("⚠️  WARNING: Pin %d (DRUM_PIN) doesn't support interrupts!\n", DRUM_PIN);
  } else {
    Serial.printf("✅ Pin %d (DRUM_PIN) supports interrupts\n", DRUM_PIN);
  }
  Serial.printf("✅ I2C pins: SDA=%d, SCL=%d\n", I2C_SDA, I2C_SCL);
}
uint32_t readADCmv(adc1_channel_t channel) {
  uint32_t raw = adc1_get_raw(channel);
  return esp_adc_cal_raw_to_voltage(raw, &adc1_chars);
}

//Patch3
void input_processing(void) {
  VCU_ready = digitalRead(36);  // Is VCU ON? True when RB+ = 12V (limited to max 3.3V on D4 input)
  sw = (digitalRead(35) || recording);
  // A/D conversion on 5 channels, 12bit resolution 0-3300mV
  mv0 = readADCmv(ADC_CH0);
  mv1 = readADCmv(ADC_CH1);
  mv2 = readADCmv(ADC_CH2);
  mv3 = readADCmv(ADC_CH3);
  mv4 = readADCmv(ADC_CH4);
  h_reg[0] = setPoint;
  setPoint_r = setPoint * GEARING;  // SetPoint pot1 ()
  h_reg[1] = (float)mv1 / 33.0;     // Throttle pot2
  tachogen = mv2 * 5.454545;        // Tacho pin 3
  h_reg[2] = tachogen;              // tachogen, Tacho signal 1023 * 17.595 = 18000 rpm
  tachogen_r = tachogen * GEARING;  // tachogen_r = tachogen * 11 / 16
//  h_reg[3] = current.torque * 100;  // Torque
  h_reg[4] = (float)mv4;            // KP adjust (pot3) format x.xx, range 0.0 => 1.0
  KP = h_reg[4];                    // red
  h_reg[5] = KI;                    // (float)analogRead(6) * 0.5; // KI adjust (pot4)
  h_reg[6] = KD = 0;
  ;  // (double)analogRead(7);  //KD HVI
  h_reg[7] = 0;
  if (setPoint == 0) h_reg[9] = 0;
  else h_reg[9] = leafFb.torqueNm;                            // closed loop b_Torque PWM (D/A) output to inverter 0 - 4095 eq 0Nm to 50Nm // Torque@Testmotor, 50[dyno_Nm max] * 11 /16 = 34.37[engine_Nm]
  h_reg[10] = 2 * Pi / 60000 * h_reg[0] * h_reg[3];  // Power [KW]
  ledcWrite(0, h_reg[1]);                     // PWM throttle output to throttle servo 0-180 deg
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Note: The b_Torq output from the PID controller has the dynamic span -1024 to +1023.
  The outputs from the set_point ADC0 and ADC1 only have a positive span from 0 1024. The b_Torq must be converted to this level!
  This means that negative b_Torq output = brake signal VCU_brake, ADC0 is inverted and limited to a span of 0 - 1024. The VCU_throttle signal is limited to the positive range 0 - 4095when state is motor.
  When VCU_brake > 0, the VCU_throttle is neglected. Braking has higher priority than acceleration!
  In state idle, a constant idle_brake torque is provided by the VCU. In this idle state, the RB- relay is ON and the idle brake energy is dumped into the brake-resistor.
  Likewise in dyno state, the RB- relay is ON and the brake energy is dumped into the brake-resistor.
  In the states motor and start, the RB- relay is OFF, the power-dump resistor from the 320V power supply, which then only is active during precharge and motoring.
  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

// Update statemachine with actual state: start, idle, motor, dyno
void get_state(void) {



  /*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  Note: The b_Torq output from the PID controller has the dynamic span -1024 to +1023.
  The outputs from the set_point ADC0 and ADC1 only have a positive span from 0 1024. The b_Torq must be converted to this level!
  This means that negative b_Torq output = brake signal VCU_brake, ADC0 is inverted and limited to a span of 0 - 1024. The VCU_throttle signal is limited to the positive range 0 - 4095when state is motor.
  When VCU_brake > 0, the VCU_throttle is neglected. Braking has higher priority than acceleration!
  In state idle, a constant idle_brake torque is provided by the VCU. In this idle state, the RB- relay is ON and the idle brake energy is dumped into the brake-resistor.
  Likewise in dyno state, the RB- relay is ON and the brake energy is dumped into the brake-resistor.
  In the states motor and start, the RB- relay is OFF, the power-dump resistor from the 320V power supply, which then only is active during precharge and motoring.
  ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
  prev_state = state;  //remember this state
                       // if (!VCU_ready) state = start;                         // while VCU not ready (!RB+), state = start
  if (setPoint <= 100 && rpm <= 100) state = State::Start;

  switch (state) {
    case State::Start:
      strip.setPixelColor(0, strip.Color(255, 255, 255));  // White
      ledcWrite(1, h_reg[9]);                       // VCU_throttle set =OFF
      ledcWrite(2, 0);                              // VCU_brake set = OFF
      digitalWrite(39, 0);                                 // SSR OFF
      if (prev_state != State::Start) delay(15);                  // Break before make wait 20 ms for RB- ( Typical relay open time  max 10ms, SSR Turn‑on delay: 0–10 ms)
      digitalWrite(38, 0);                                 // RB- = OFF
      if (setPoint >= 200 && setPoint <= 2000) state = State::Motor;
      else {
        if (setPoint > 2000 && rpm >= setPoint) state = State::Dyno;
      }
      break;

    case State::Idle:
      strip.setPixelColor(0, strip.Color(0, 0, 255));  // Blue
      ledcWrite(1, 0);                          // VCU_throttle set = OFF
      ledcWrite(2, 0);                          // VCU_brake set = OFF.    NB! idle regen brake setup in VCU, to about 0 - 5 Nm
      digitalWrite(39, 0);                             // SSR OFF
      if (prev_state != State::Idle) delay(15);               // Break before make wait 20 ms for RB- ( Typical relay open time  max 10ms, SSR Turn‑on delay: 0–10 ms)
      digitalWrite(38, 1);                             // RB- = ON
      if (rpm < 200) state = State::Start;
      break;

    case State::Dyno:
      strip.setPixelColor(0, strip.Color(0, 255, 0));  // Green
      ledcWrite(1, 0);                          // VCU_throttle set = OFF
      ledcWrite(2, -h_reg[9]);                  // VCU_brake set = ON , notice h_reg[9] change sign to positive brake signal to VCU_brake
      digitalWrite(39, 0);                             // SSR OFF
      if (prev_state != State::Dyno) delay(15);               // Break before make wait 20 ms for RB- ( Typical relay open time  max 10ms, SSR Turn‑on delay: 0–10 ms)
      digitalWrite(38, 1);                             // RB- = ON
      if (setPoint <= 2000) state = State::Idle;
      break;

    case State::Motor:
      strip.setPixelColor(0, strip.Color(255, 0, 0));  // Red
      ledcWrite(1, h_reg[9]);                   // VCU_throttle set = ON. NB! Throttel is ignored if VCU brake is ON
      ledcWrite(2, 0);                          // VCU_brake set = OFF
      digitalWrite(38, 0);                             // RB- = OFF
      if (prev_state != State::Motor) delay(15);              // Break before make wait 20 ms for RB- ( Typical relay open time  max 10ms, SSR Turn‑on delay: 0–10 ms)
      digitalWrite(39, 1);                             // SSR ON
      if (setPoint < 200) state = State::Start;
      if (rpm > 2000) state = State::Dyno;
      break;

    default:
      state = State::Start;
      break;
  }
  //myPID.setGains(KP, KI, KD);  // Manually adjust of PID parameters by potentiometers
  //myPID.run();                 // call in every loop
  strip.show();
}

// Schedule and control of rpm rampup
void scheduler(void) {
  float man_set = 0;
  // When switch ON, start auto RPM rampup sequence
  if (sw && autoMode) rampup = true;
  else rampup = false;
  if (rampup && setPoint <= autoEndRPM) {
    setPoint += 21.33333333;  // Increase rpm setpoint
    if ((setPoint >= 7000) && (setPoint < autoEndRPM)) h_reg[8] += (h_reg[10] * 1e-2);
  }
  if (sw && !rampup && setPoint > autoStartRPM) {
    setPoint -= 64.0;  // Increase rpm setpoint
    if (setPoint < autoStartRPM) setPoint = autoStartRPM;
  }

  if (!sw) {
    man_set = mv0 * 5.454545;
    // Limit manual dn/dt slope of setPoint to 300 rpm/s (max = 1000/s)
    if (man_set > setPoint + 60.0) man_set = setPoint + 60.0;  // delta RPM = 191 -> dn/de <= 1000 RPM
    if (man_set < setPoint - 60.0) man_set = setPoint - 60.0;
    if ((state == State::Motor) && (man_set >= setPoint_max)) man_set = setPoint_max;  // limit max rpm to 2000 rpm in motor state
    h_reg[0] = setPoint = man_set;
    rampup = false;
    sign = 0;
    h_reg[8] = 0;
  }
}

// print the SSID of the network you're attached to:
void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
//Patch3 end

// ─── Inertia calculation ─────────────────────────────────────────
float calculateDrumInertia() {
  float radius = DRUM_DIAM_METERS / 2.0f;
  if (DRUM_INERTIA_TYPE == "solid") return 0.5f * DRUM_MASS * radius * radius;
  if (DRUM_INERTIA_TYPE == "hollow") {
    float inner_r = radius - (DRUM_WALL_THICKNESS_CM / 100.0f);
    if (inner_r <= 0) inner_r = 0.01f;
    return 0.5f * DRUM_MASS * (radius * radius + inner_r * inner_r);
  }
  return DRUM_INERTIA_CUSTOM;
}

// ─── Save full run ───────────────────────────────────────────────
void saveFullRun() {
  if (currentRun.empty()) {
    Serial.println("⚠️ No data points to save!");
    return;
  }

  Serial.printf("💾 Saving run with %d data points\n", currentRun.size());

  String filename = "/run_" + String(millis()) + ".csv";
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file.println("RPM,HP,Torque");

  size_t targetPoints = 900;
  int step = (currentRun.size() > targetPoints) ? (currentRun.size() / targetPoints) : 1;
  for (size_t i = 0; i < currentRun.size(); i += step) {
    file.println(String(currentRun[i].rpm) + "," + String(currentRun[i].hp, 2) + "," + String(currentRun[i].torque, 2));
  }
  file.close();

  File idxFile = LittleFS.open("/runs.idx", "a");
  if (idxFile) {
    idxFile.println(String(millis()) + "," + filename + "," + currentCustomer + "," + currentUnit + "," + String(runPeakHP, 2) + "," + String(runPeakTorque, 2) + "," + String(egtTracker.peak, 0));
    idxFile.close();
  }

  loadRunList();
  while (savedRuns.size() > MAX_SAVED_RUNS) {
    RunInfo oldest = savedRuns.back();
    LittleFS.remove(oldest.filename);
    savedRuns.pop_back();
  }

  File newIdx = LittleFS.open("/runs.idx.tmp", "w");
  for (const auto &run : savedRuns) {
    newIdx.println(String(run.timestamp) + "," + run.filename + "," + run.customer + "," + run.vehicle + "," + String(run.peakHP, 2) + "," + String(run.peakTorque, 2) + "," + String(run.peakEGT, 0));
  }
  newIdx.close();
  LittleFS.remove("/runs.idx");
  LittleFS.rename("/runs.idx.tmp", "/runs.idx");
}

// ─── Load run list ───────────────────────────────────────────────
void loadRunList() {
  savedRuns.clear();
  File idxFile = LittleFS.open("/runs.idx", "r");
  if (!idxFile) return;

  while (idxFile.available()) {
    String line = idxFile.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 + 1);
    int comma3 = line.indexOf(',', comma2 + 1);
    int comma4 = line.indexOf(',', comma3 + 1);
    int comma5 = line.indexOf(',', comma4 + 1);
    int comma6 = line.indexOf(',', comma5 + 1);

    if (comma1 > 0 && comma2 > 0) {
      RunInfo run;
      run.timestamp = line.substring(0, comma1).toInt();
      run.filename = line.substring(comma1 + 1, comma2);
      run.customer = line.substring(comma2 + 1, comma3);
      run.vehicle = line.substring(comma3 + 1, comma4);
      run.peakHP = line.substring(comma4 + 1, comma5).toFloat();
      run.peakTorque = line.substring(comma5 + 1, comma6).toFloat();
      run.peakEGT = line.substring(comma6 + 1).toFloat();
      savedRuns.push_back(run);
    }
  }
  idxFile.close();
}

// ─── Send run list ───────────────────────────────────────────────
void sendRunList() {
  String json = "{\"type\":\"runs\",\"data\":[";
  for (size_t i = 0; i < savedRuns.size(); i++) {
    if (i > 0) json += ",";
    json += "{\"timestamp\":" + String(savedRuns[i].timestamp) + ",\"filename\":\"" + savedRuns[i].filename + "\"" + ",\"customer\":\"" + savedRuns[i].customer + "\"" + ",\"vehicle\":\"" + savedRuns[i].vehicle + "\"" + ",\"hp\":" + String(savedRuns[i].peakHP, 1) + ",\"torque\":" + String(savedRuns[i].peakTorque, 1) + ",\"egt\":" + String(savedRuns[i].peakEGT, 0) + "}";
  }
  json += "]}";
  MetaSense::ws.textAll(json);
}

// ─── Send run data ───────────────────────────────────────────────
void sendRunData(String filename) {
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("File not found: " + filename);
    return;
  }

  String json = "{\"type\":\"run_data\",\"filename\":\"" + filename + "\",\"points\":[";
  bool first = true;

  file.readStringUntil('\n');

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int comma1 = line.indexOf(',');
    int comma2 = line.indexOf(',', comma1 + 1);

    if (comma1 > 0 && comma2 > 0) {
      if (!first) json += ",";
      float rpm = line.substring(0, comma1).toFloat();
      float hp = line.substring(comma1 + 1, comma2).toFloat();
      float torque = line.substring(comma2 + 1).toFloat();
      json += "{\"r\":" + String(rpm, 0) + ",\"h\":" + String(hp, 2) + ",\"t\":" + String(torque, 2) + "}";
      first = false;
    }
  }
  json += "]}";
  file.close();
  MetaSense::ws.textAll(json);
}

// ─── Delete run ──────────────────────────────────────────────────
void deleteRun(String filename, int index) {
  LittleFS.remove(filename);
  if (index >= 0 && index < (int)savedRuns.size()) {
    savedRuns.erase(savedRuns.begin() + index);
  }
  File idxFile = LittleFS.open("/runs.idx", "w");
  for (const auto &run : savedRuns) {
    idxFile.println(String(run.timestamp) + "," + run.filename + "," + run.customer + "," + run.vehicle + "," + String(run.peakHP, 2) + "," + String(run.peakTorque, 2) + "," + String(run.peakEGT, 0));
  }
  idxFile.close();
}

// ─── Broadcast settings ──────────────────────────────────────────
void broadcastSettings() {
  String json = "{";
  json += "\"type\":\"settings\",";
  json += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
  json += "\"buildDate\":\"" + String(BUILD_DATE) + "\",";
  json += "\"maxRPM\":" + String(MAX_RPM, 0) + ",";
  json += "\"maxHP\":" + String(MAX_HP, 1) + ",";
  json += "\"maxTorque\":" + String(MAX_TORQUE, 1) + ",";
  json += "\"armCm\":" + String(LEVER_ARM_METERS * 100, 1) + ",";
  json += "\"calFactor\":" + String(CALIBRATION_FACTOR, 6) + ",";
  json += "\"pulsesPerRev\":" + String(PULSES_PER_REVOLUTION, 1) + ",";
  json += "\"brakeToEngineRatio\":" + String(BRAKE_TO_ENGINE_RATIO, 2) + ",";
  json += "\"rpmFilter\":" + String(RPM_FILTER_ALPHA, 2) + ",";
  json += "\"mode\":\"" + DYNO_MODE + "\",";
  json += "\"drumMass\":" + String(DRUM_MASS, 1) + ",";
  json += "\"drumCm\":" + String(DRUM_DIAM_METERS * 100, 1) + ",";
  json += "\"pulsesPerRevDrum\":" + String(PULSES_PER_REV_DRUM, 1) + ",";
  json += "\"drumInertiaType\":\"" + DRUM_INERTIA_TYPE + "\",";
  json += "\"drumWallCm\":" + String(DRUM_WALL_THICKNESS_CM, 1) + ",";
  json += "\"drumInertiaCustom\":" + String(DRUM_INERTIA_CUSTOM, 3) + ",";
  json += "\"rpmSource\":\"" + RPM_SOURCE + "\",";
  json += "\"virtGearRatio\":" + String(VIRTUAL_GEAR_RATIO, 2) + ",";
  json += "\"energy\":" + String(h_reg[8], 2) + ",";
  json += "\"drivetrainEff\":" + String(DRIVETRAIN_EFFICIENCY * 100, 1);
  json += "}";
  MetaSense::ws.textAll(json);
}
/*
// ─── Send live data (EGT FIXED) ──────────────────────────────────
// ─── Broadcast settings ──────────────────────────────────────────
void notifyClients(const Telemetry &data, bool isRecording) {
  String json;
  json.reserve(768);

  json = "{";
  json += "\"type\":\"data\",";

  // Dyno core
  json += "\"rpm\":" + String(data.rpm, 0) + ",";
  json += "\"drum_rpm\":" + String(h_reg[1], 0) + ",";
  json += "\"hp\":" + String(data.hp, 2) + ",";
  json += "\"torque\":" + String(data.torque, 2) + ",";
  json += "\"brakeTorque\":" + String(data.brakeTorque, 2) + ",";
  json += "\"load_kg\":" + String(setPoint, 0) + ",";
  json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
  json += "\"peakHP\":" + String(data.peakHP, 2) + ",";
  json += "\"peakHP_RPM\":" + String(data.peakHP_RPM, 0) + ",";
  json += "\"peakTorque\":" + String(data.peakTorque, 2) + ",";
  json += "\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0) + ",";
  json += "\"air_density\":" + String(air_density, 3) + ",";
  json += "\"ambient_temp\":" + String(ambient_temp_c, 1) + ",";
  json += "\"pressure\":" + String(atmospheric_pressure, 1) + ",";
  json += "\"gear_ratio\":" + String(e_torque, 0) + ",";
  json += "\"energy\":" + String((h_reg[8] / 1000), 2) + ",";
  json += "\"ratio_confidence\":" + String((h_reg[13]) / 100.0, 2) + ",";
  json += "\"dyno_mode\":\"" + DYNO_MODE + "\",";

  // Leaf inverter block
  json += "\"leaf\":{";
  json += "\"rpm\":" + String(data.leaf_rpm, 0) + ",";
  json += "\"torque\":" + String(data.leaf_torqueNm, 2) + ",";
  json += "\"inv_temp\":" + String(data.leaf_invTempC, 1) + ",";
  json += "\"motor_temp\":" + String(data.leaf_motorTempC, 1) + ",";
  json += "\"coolant_temp\":" + String(data.leaf_coolantTempC, 1) + ",";
  json += "\"inv_ready\":" + String(data.leaf_invReady ? "true" : "false") + ",";
  json += "\"inv_fault\":" + String(data.leaf_invFault ? "true" : "false") + ",";
  json += "\"hv_interlock_ok\":" + String(data.leaf_hvInterlockOk ? "true" : "false") + ",";
  json += "\"gear_state\":" + String(data.leaf_gearState, 0);
  json += "},";

  // EGT block (unchanged, just moved down)
  json += "\"mcps\":{";
  if (mcp_egt_ok) {
    float hot = egtTracker.getFiltered();
    float amb = egtTracker.getAmbient();
    if (hot > 0 && hot < 1500) {
      json += "\"7\":{\"hot\":" + String(hot, 1) + ",\"amb\":" + String(amb, 1) + "}";
    } else {
      json += "\"7\":{\"hot\":0,\"amb\":0}";
    }
    h_reg[11] = amb * 10;
    h_reg[12] = hot * 10;
  }
  json += "}";

  json += "}";
  ws.textAll(json);
}

/*
void notifyClients(const LiveData &data, bool isRecording) {
  String json;
  json.reserve(512);

  json = "{";
  json += "\"type\":\"data\",";
  json += "\"rpm\":" + String(data.rpm, 0) + ",";
  json += "\"drum_rpm\":" + String(h_reg[1], 0) + ",";  // shows Engine throttle
  json += "\"hp\":" + String(data.hp, 2) + ",";         // Shows kW
  json += "\"torque\":" + String(data.torque, 2) + ",";
  json += "\"brakeTorque\":" + String(data.brakeTorque, 2) + ",";
  json += "\"load_kg\":" + String(setPoint, 0) + ",";  // shows setPoint RPM
  json += "\"recording\":" + String(isRecording ? "true" : "false") + ",";
  json += "\"peakHP\":" + String(data.peakHP, 2) + ",";
  json += "\"peakHP_RPM\":" + String(data.peakHP_RPM, 0) + ",";
  json += "\"peakTorque\":" + String(data.peakTorque, 2) + ",";
  json += "\"peakTorque_RPM\":" + String(data.peakTorque_RPM, 0) + ",";
  json += "\"air_density\":" + String(air_density, 3) + ",";
  json += "\"ambient_temp\":" + String(ambient_temp_c, 1) + ",";
  json += "\"pressure\":" + String(atmospheric_pressure, 1) + ",";
  json += "\"gear_ratio\":" + String(e_torque, 0) + ",";       // Shows E_torque
  json += "\"energy\":" + String((h_reg[8] / 1000), 2) + ",";  //Shows energy
  //json += "\"ratio_confidence\":" + String(gearTracker.getConfidence(), 2) + ",";
  json += "\"ratio_confidence\":" + String((h_reg[13]) / 100, 2) + ",";  // Shows Relative humidity
  json += "\"dyno_mode\":\"" + DYNO_MODE + "\",";

  // FIXED EGT SECTION - matches original format exactly
  json += "\"mcps\":{";
  if (mcp_egt_ok) {
    float hot = egtTracker.getFiltered();
    float amb = egtTracker.getAmbient();
    if (hot > 0 && hot < 1500) {  // Valid EGT range
      json += "\"7\":{\"hot\":" + String(hot, 1) + ",\"amb\":" + String(amb, 1) + "}";
    } else {
      // Sensor exists but no valid reading yet
      json += "\"7\":{\"hot\":0,\"amb\":0}";
    }

    //patch
    h_reg[11] = amb * 10;
    h_reg[12] = hot * 10;
  }
  json += "}";
  json += "}";
  ws.textAll(json);
}
*/
// ─── Send run completion ─────────────────────────────────────────
void sendRunComplete(float peakHP, float peakHP_RPM, float peakTorque, float peakTorque_RPM) {
  saveFullRun();

  String json = "{";
  json += "\"type\":\"run_complete\",";
  json += "\"peakHP\":" + String(peakHP, 2) + ",";
  json += "\"peakHP_RPM\":" + String(peakHP_RPM, 0) + ",";
  json += "\"peakTorque\":" + String(peakTorque, 2) + ",";
  json += "\"peakTorque_RPM\":" + String(peakTorque_RPM, 0) + ",";
  json += "\"peakEGT\":" + String(egtTracker.peak, 0) + ",";
  json += "\"finalEGT\":" + String(egtTracker.final, 0) + ",";
  json += "\"ambient\":" + String(egtTracker.ambient, 1);
  //  json += "\"energy\":" + String(h_reg[8]/1000, 2);
  json += "}";
  MetaSense::ws.textAll(json);

  egtTracker.reset();
  sendRunList();
}

// ─── Save smoothed run ───────────────────────────────────────────
void saveSmoothedRun(String customer, String unit, String comments,
                     float peakHP, float peakHP_RPM, float peakTorque, float peakTorque_RPM, float peakEGT,
                     String pointsJson) {
  String filename = "/run_" + String(millis()) + ".csv";
  File file = LittleFS.open(filename, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  file.println("RPM,HP,Torque");

  int pointsStart = pointsJson.indexOf("\"points\":[") + 9;
  int pointsEnd = pointsJson.lastIndexOf("]}");
  String pointsStr = pointsJson.substring(pointsStart, pointsEnd);

  int pos = 0;
  int pointCount = 0;
  while (pos < pointsStr.length()) {
    int rStart = pointsStr.indexOf("\"r\":", pos);
    if (rStart == -1) break;
    int rEnd = pointsStr.indexOf(",", rStart);
    float rpm = pointsStr.substring(rStart + 4, rEnd).toFloat();

    int hStart = pointsStr.indexOf("\"h\":", rEnd);
    int hEnd = pointsStr.indexOf(",", hStart);
    float hp = pointsStr.substring(hStart + 4, hEnd).toFloat();

    int tStart = pointsStr.indexOf("\"t\":", hEnd);
    int tEnd = pointsStr.indexOf("}", tStart);
    float torque = pointsStr.substring(tStart + 4, tEnd).toFloat();

    file.println(String(rpm) + "," + String(hp, 2) + "," + String(torque, 2));
    pointCount++;
    pos = tEnd;
  }
  file.close();

  File idxFile = LittleFS.open("/runs.idx", "a");
  if (idxFile) {
    time_t now = time(nullptr);
    if (now < 100000) now = millis() / 1000;
    idxFile.println(String(now) + "," + filename + "," + customer + "," + unit + "," + String(peakHP, 2) + "," + String(peakTorque, 2) + "," + String(peakEGT, 0));
    idxFile.close();
  }
  loadRunList();
  sendRunList();
}

// ─── Calibration Task ────────────────────────────────────────────
void calibrationTask(void *pvParameters) {
  for (;;) {
    if (calibRequest) {
      calibRequest = false;
      MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Calibrating... hold weight still for 5 seconds (tare first!)\"}");
      vTaskDelay(pdMS_TO_TICKS(2000));

      long raw_sum = 0;
      int valid_count = 0;
      const int skip_first_n = 10;
      const unsigned long durationMs = 5000UL;
      unsigned long startTime = millis();
      while (millis() - startTime < durationMs) {
        if (nau_initialized && nau.available()) {
          long current = nau.read() - zeroOffset;
          if (labs(current) > 100000) {
            if (valid_count >= skip_first_n) raw_sum += current;
            valid_count++;
          }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      int averaged_samples = valid_count - skip_first_n;
      if (averaged_samples < 5) {
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Not enough stable readings. Try again.\"}");
      } else {
        double avgRaw = (double)raw_sum / averaged_samples;
        CALIBRATION_FACTOR = (calibKnownWeight * 1000.0f) / avgRaw;
        preferences.putFloat("cal_factor", CALIBRATION_FACTOR);
        broadcastSettings();
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Calibration done! New factor: " + String(CALIBRATION_FACTOR, 6) + "\"}");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ─── WebSocket Event Handler ─────────────────────────────────────
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    String msg = "";
    for (size_t i = 0; i < len; i++) msg += (char)data[i];
    msg.trim();

    if (msg == "TARE") {
      MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Taring... remove all load\"}");
      vTaskDelay(pdMS_TO_TICKS(1000));
      long sum = 0;
      int count = 0;
      for (int i = 0; i < 16; i++) {
        if (nau_initialized && nau.available()) {
          sum += nau.read();
          count++;
          vTaskDelay(pdMS_TO_TICKS(5));
        }
      }
      if (count > 0) {
        zeroOffset = sum / count;
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Tare complete\"}");
      } else {
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Tare failed\"}");
      }
      return;
    }
/*
    if (msg == "RESET_RATIO") {
      gearTracker.reset();
      MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Gear ratio tracking reset\"}");
      return;
    }
*/
    if (msg.startsWith("SET_CUSTOMER:")) {
      String data = msg.substring(13);
      int pipe1 = data.indexOf('|');
      int pipe2 = data.indexOf('|', pipe1 + 1);
      if (pipe1 > 0 && pipe2 > 0) {
        currentCustomer = data.substring(0, pipe1);
        currentUnit = data.substring(pipe1 + 1, pipe2);
        currentComments = data.substring(pipe2 + 1);
      }
      return;
    }

    if (msg.startsWith("CALIBRATE:")) {
      float knownWeightKg = msg.substring(10).toFloat();
      if (knownWeightKg <= 0) {
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Enter valid weight > 0\"}");
        return;
      }
      calibKnownWeight = knownWeightKg;
      calibRequest = true;
      MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Calibration starting...\"}");
      return;
    }

    if (msg.startsWith("SET_SINGLE:")) {
      String part = msg.substring(11);
      int eq = part.indexOf('=');
      if (eq == -1) return;

      String key = part.substring(0, eq);
      String valStr = part.substring(eq + 1);
      float val = valStr.toFloat();
      if (key == "maxRPM" && val >= 5000 && val <= 50000) MAX_RPM = val;
      else if (key == "maxHP" && val >= 5 && val <= 100) MAX_HP = val;
      else if (key == "maxTorque" && val >= 5 && val <= 100) MAX_TORQUE = val;
      else if (key == "armCm" && val >= 5 && val <= 100) {
        LEVER_ARM_METERS = val / 100.0f;
        preferences.putFloat("lever_arm", LEVER_ARM_METERS);
      } else if (key == "pulsesPerRev" && val >= 0.5f && val <= 10.0f) {
        PULSES_PER_REVOLUTION = val;
        preferences.putFloat("pulses_per_rev", val);
      } else if (key == "brakeToEngineRatio" && val >= 0.1f && val <= 50.0f) {
        BRAKE_TO_ENGINE_RATIO = val;
        preferences.putFloat("brake_eng_ratio", val);
      } else if (key == "rpmFilter" && val >= 0.1f && val <= 0.9f) {
        RPM_FILTER_ALPHA = val;
        preferences.putFloat("rpm_filter", val);
      } else if (key == "autoHyst" && val >= 50 && val <= 500) {
        AUTO_HYSTERESIS = val;
        preferences.putFloat("auto_hyst", val);
      } else if (key == "dynoMode") {
        if (valStr == "brake" || valStr == "inertia") {
          DYNO_MODE = valStr;
          preferences.putString("dyno_mode", DYNO_MODE);
        }
      } else if (key == "drumMass" && val >= 1.0f && val <= 300.0f) {
        DRUM_MASS = val;
        preferences.putFloat("drum_mass", DRUM_MASS);
      } else if (key == "drumCm" && val >= 10.0f && val <= 300.0f) {
        DRUM_DIAM_METERS = val / 100.0f;
        preferences.putFloat("drum_diam", DRUM_DIAM_METERS);
      } else if (key == "pulsesPerRevDrum" && val >= 0.5f && val <= 20.0f) {
        PULSES_PER_REV_DRUM = val;
        preferences.putFloat("pulses_drum", PULSES_PER_REV_DRUM);
      } else if (key == "drumInertiaType") {
        if (valStr == "solid" || valStr == "hollow" || valStr == "custom") {
          DRUM_INERTIA_TYPE = valStr;
          preferences.putString("drum_inertia_type", DRUM_INERTIA_TYPE);
        }
      } else if (key == "drumWallCm" && val >= 0.5f && val <= 20.0f) {
        DRUM_WALL_THICKNESS_CM = val;
        preferences.putFloat("drum_wall_cm", DRUM_WALL_THICKNESS_CM);
      } else if (key == "drumInertiaCustom" && val >= 0.01f && val <= 100.0f) {
        DRUM_INERTIA_CUSTOM = val;
        preferences.putFloat("drum_inertia_custom", DRUM_INERTIA_CUSTOM);
      } else if (key == "drivetrainEff" && val >= 70.0f && val <= 99.0f) {
        DRIVETRAIN_EFFICIENCY = val / 100.0f;
        preferences.putFloat("drivetrain_eff", DRIVETRAIN_EFFICIENCY);
      } else if (key == "rpmSource") {
        if (valStr == "spark" || valStr == "brake" || valStr == "drum" || valStr == "cvt") {
          RPM_SOURCE = valStr;
          preferences.putString("rpm_source", RPM_SOURCE);
          if (valStr == "cvt") {
            VIRTUAL_GEAR_RATIO = 1.0f;
            preferences.putFloat("virt_gear_ratio", 1.0f);
          }
        }
      } else if (key == "virtGearRatio" && val >= 0.1f && val <= 200.0f) {
        if (RPM_SOURCE != "cvt" && DYNO_MODE != "brake") {
          VIRTUAL_GEAR_RATIO = val;
          preferences.putFloat("virt_gear_ratio", VIRTUAL_GEAR_RATIO);
        }
      }
      broadcastSettings();
      return;
    }

    if (msg.startsWith("WIFI_CLIENT:")) {
      int split = msg.indexOf(':', 12);
      if (split != -1) {
        sta_ssid = msg.substring(12, split);
        sta_password = msg.substring(split + 1);
        preferences.putString("sta_ssid", sta_ssid);
        preferences.putString("sta_password", sta_password);
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"WiFi credentials saved. Rebooting...\"}");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
      }
      return;
    }

    if (msg == "RESET_DEFAULT") {
      preferences.clear();
      MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Reset to default AP mode. Rebooting...\"}");
      vTaskDelay(pdMS_TO_TICKS(1000));
      ESP.restart();
      return;
    }

    if (msg == "RESET_ALL_SETTINGS") {
      preferences.clear();
      LittleFS.format();
      MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"All settings and runs reset. Rebooting...\"}");
      vTaskDelay(pdMS_TO_TICKS(1000));
      ESP.restart();
      return;
    }

    if (msg.startsWith("AUTO_RUN:")) {
      int colon1 = msg.indexOf(':', 9);
      if (colon1 != -1) {
        autoStartRPM = msg.substring(9, colon1).toFloat();
        autoEndRPM = msg.substring(colon1 + 1).toFloat();
        if (autoStartRPM > 0 && autoEndRPM > autoStartRPM) {
          autoMode = true;
          MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Auto Run activated\"}");
        } else {
          MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Invalid RPM range\"}");
        }
      }
      return;
    }

    if (msg == "GET_SETTINGS") {
      broadcastSettings();
      return;
    }

    if (msg == "GET_RUNS") {
      sendRunList();
      return;
    }

    if (msg.startsWith("GET_RUN_DATA:")) {
      String filename = msg.substring(13);
      sendRunData(filename);
      return;
    }

    if (msg.startsWith("DELETE_RUN:")) {
      int idx = msg.substring(11).toInt();
      if (idx >= 0 && idx < (int)savedRuns.size()) {
        deleteRun(savedRuns[idx].filename, idx);
        sendRunList();
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Run deleted\"}");
      }
      return;
    }

    if (msg == "PAGE_SETTINGS") {
      return;
    }

    if (msg == "PAGE_TRENDS") {
      return;
    }

    if (msg == "PAGE_MAIN") {
      return;
    }

    if (msg == "CANCEL_AUTO_RUN") {
      if (autoMode || recording) {
        autoMode = false;
        if (recording) {
          recording = false;
          recordingStartedMs = 0;
          sendRunComplete(runPeakHP, runPeakHP_RPM, runPeakTorque, runPeakTorque_RPM);
        }
        currentRun.clear();
        runPeakHP = 0;
        runPeakHP_RPM = 0;
        runPeakTorque = 0;
        runPeakTorque_RPM = 0;
        egtTracker.reset();
        hpSmooth.reset();
        torqueSmooth.reset();
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Run cancelled\"}");
      }
      return;
    }

    if (msg == "MANUAL_START") {
      if (!recording) {
        recording = true;
        autoMode = false;
        currentRun.clear();
        runPeakHP = 0;
        runPeakHP_RPM = 0;
        runPeakTorque = 0;
        runPeakTorque_RPM = 0;
        egtTracker.reset();
        hpSmooth.reset();
        torqueSmooth.reset();
        recordingStartedMs = millis();
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Manual recording started\"}");
      } else {
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Recording already active\"}");
      }
      return;
    }

    if (msg == "MANUAL_STOP") {
      if (recording) {
        recording = false;
        autoMode = false;
        recordingStartedMs = 0;
        sendRunComplete(runPeakHP, runPeakHP_RPM, runPeakTorque, runPeakTorque_RPM);
      } else {
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"No active recording\"}");
      }
      return;
    }
  }
}

// ─── Captive Portal Handler ─────────────────────────────────────
class CaptiveRequestHandler : public AsyncWebHandler {
public:
  bool canHandle(AsyncWebServerRequest *request) {
    String url = request->url();
    if (url == "/" || url == "/ws" || url == "/favicon.ico" || url == "/version" || url == "/settings" || url == "/trend.html" || url == "/update" || url == "/update_fs") {
      return false;
    }
    return !LittleFS.exists(url);
  }
  void handleRequest(AsyncWebServerRequest *request) {
    request->redirect("/");
  }
};

// ─── Sensor Task (OPTIMIZED + EGT FIXED) ─────────────────────────
void sensorTask(void *parameter) {
  static float engine_rpm_raw = 0.0f;
  static float engine_rpm_filtered = 0;
  static float drum_rpm_filtered = 0.0f;
  static float last_valid_engine_rpm = 0;
  static float last_valid_drum_rpm = 0;

  static unsigned long last_ws_update = 0;
  static unsigned long last_egt_read = 0;
  static unsigned long last_gear_calc = 0;
  static unsigned long last_bmp_read = 0;

  static float last_drum_rpm = 0.0f;
  static unsigned long last_accel_time = 0;
  static float accel_filtered = 0.0f;

  // EGT retry counter
  static int egt_retry_count = 0;
  static bool egt_first_read = true;
  for (;;) {
    unsigned long now = millis();

    if (!settingsPageActive) {

      // CAN RX
      twai_message_t rx;
      if (twai_receive(&rx, 0) == ESP_OK)
        leafCanDecode(rx);
//notifyClients(live, isRecording);
      // Read sensors
      float rpmMeas = tachogen;//readRpmFromAdcMv(readRpmAdcMv());
//      float idcMeas = readIdcFromAdcMv(readIdcAdcMv());

//      updateFaultState(rpmMeas, idcMeas);
      resetFault();

      float rpmSet = rpmRamp.update();

      float omega = rpmSet * (2.0f * M_PI / 60.0f);
      float domega = (omega - lastOmega) / dt_ctrl;
      lastOmega = omega;

      float Tff = J * domega;
      float Tpi = speedPI.update(rpmSet, rpmMeas);

      float torqueCmd = Tff + Tpi;
      if (torqueCmd > TORQUE_MAX) torqueCmd = TORQUE_MAX;
      if (torqueCmd < TORQUE_MIN) torqueCmd = TORQUE_MIN;

      float safeTorque = applySafetyGate(torqueCmd);

      if (now - lastInvUpdateMs >= 100) {
        float maxStep = TORQUE_SLEW * dt_inv;

        float diff = safeTorque - torqueHold;
        if (diff > maxStep) diff = maxStep;
        if (diff < -maxStep) diff = -maxStep;

        torqueHold += diff;
//        sendTorqueToInverter(torqueHold);
//        sendCanTorque(torqueHold);

        lastInvUpdateMs = now;
      }

      //Patch4
      input_processing();
      get_state();
      scheduler();
      //Patch4 end

      // ─── LOAD CELL ────────────────────────────────────────────────
      float grams = 0.0f;
      float kg = 0.0f;
      float force_N = 0.0f;
      static float torque_filtered = 0;
      static float b_torque_filtered = 0;
      if (nau_initialized && nau.available()) {
        long raw = nau.read();
        grams = (float)(raw - zeroOffset) * CALIBRATION_FACTOR;
      }
      kg = grams / 1000.0;
      if (kg < 0) kg = 0;
//      current.load_kg = kg;
      force_N = kg * 9.81f;
      //patch 4.1
      Torque = RPM_FILTER_ALPHA * force_N + (1.0f - RPM_FILTER_ALPHA) * Torque;
      Torque = torque_filtered;
      e_torque = RPM_FILTER_ALPHA * leafFb.torqueNm + (1.0f - RPM_FILTER_ALPHA) * e_torque;

      //patch 4.1 end

      // ─── BMP280 (throttled to 2Hz) ────────────────────────────────
      if (bmp_initialized && (now - last_bmp_read >= BMP_UPDATE_MS)) {
        atmospheric_pressure = bmp.readPressure() / 100.0F;
        ambient_temp_c = bmp.readTemperature();

        float temp_kelvin = ambient_temp_c + 273.15;
        if (temp_kelvin > 0 && atmospheric_pressure > 0) {
          air_density = (atmospheric_pressure * 100.0) / (287.05 * temp_kelvin);
          air_density = constrain(air_density, 1.0, 1.3);
        }
/*
        //patch 4.2
        // ─── AHT20 Reading (Relative humidity RH) ─────────────────────────────
        if (aht_initialized) {
          aht.getEvent(&humidity, &temp_aht);  // Get AHT20 events
          h_reg[13] = humidity.relative_humidity + 8;
          float H2O_pp_correction_factor = atmospheric_pressure / atmospheric_pressure - (((h_reg[13]) / 100) * 6.108 * exp((17.27 * ambient_temp_c) / (ambient_temp_c + 273.3)));  //hPa
        }
        //patch 4.2 end
*/
        last_bmp_read = now;
      }


      // ─── SMOOTHING ───────────────────────────────────────────────
      float hp_smooth;  // = hpSmooth.add(hp);
      float torque_smooth = torqueSmooth.add(Torque);
      float correction_factor = constrain(STD_AIR_DENSITY / air_density, 0.85f, 1.15f);
      //      hp_smooth *= correction_factor;
      //      torque_smooth *= correction_factor;
      torque_smooth *= (correction_factor * H2O_pp_correction_factor);
      hp_smooth = (engine_rpm_filtered * torque_smooth) * 2.0 * 3.141593416 / 60000.0;

      if (hp_smooth > MAX_HP * 2.0f) hp_smooth = MAX_HP * 2.0f;
      if (torque_smooth > MAX_TORQUE * 2.0f) torque_smooth = MAX_TORQUE * 2.0f;

      // ─── EGT READING (FIXED - 15Hz with retry) ────────────────────
      if (mcp_egt_ok && (now - last_egt_read >= EGT_UPDATE_MS || egt_first_read)) {
        float hot = mcp_egt.readThermocouple();
        float amb = mcp_egt.readAmbient();

        // Force faster reads at startup to get first valid reading
        if (egt_first_read) {
          vTaskDelay(pdMS_TO_TICKS(10));
          hot = mcp_egt.readThermocouple();
          amb = mcp_egt.readAmbient();
          egt_first_read = false;
          Serial.println("EGT: Initial read attempted");
        }

        if (!isnan(hot) && !isnan(amb) && hot > -50 && hot < 1500) {
          egtTracker.update(hot, amb);
          egt_retry_count = 0;
          if (egtTracker.getFiltered() > 0) {
            // Only print occasionally to avoid spam
            static unsigned long last_egt_print = 0;
            if (millis() - last_egt_print > 15000) {
              //              Serial.printf("EGT: %.1f°C | Ambient: %.1f°C\n", hot, amb);
              last_egt_print = millis();
            }
          }
        } else {
          egt_retry_count++;
          if (egt_retry_count < 20) {
            // Keep trying quickly for first few attempts
            last_egt_read = now - (EGT_UPDATE_MS - 20);
          }
        }
        last_egt_read = now;
      }

      // ─── UPDATE LIVE DATA ────────────────────────────────────────

      // ─── AUTO RUN LOGIC ──────────────────────────────────────────
      if (autoMode && !recording && leafFb.rpm >= (autoStartRPM + AUTO_HYSTERESIS)) {
        recording = true;
        currentRun.clear();
        runPeakHP = 0;
        runPeakHP_RPM = 0;
        runPeakTorque = 0;
        runPeakTorque_RPM = 0;
        egtTracker.reset();
        hpSmooth.reset();
        torqueSmooth.reset();
        recordingStartedMs = millis();
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Auto Run started\"}");
      }

      if (autoMode && recording && leafFb.rpm >= (autoEndRPM - AUTO_HYSTERESIS)) {
        recording = false;
        autoMode = false;
        recordingStartedMs = 0;
        sendRunComplete(runPeakHP, runPeakHP_RPM, runPeakTorque, runPeakTorque_RPM);
      }

      if (recording && (millis() - recordingStartedMs > 45000UL)) {
        recording = false;
        autoMode = false;
        MetaSense::ws.textAll("{\"type\":\"info\",\"msg\":\"Safety timeout - run stopped\"}");
        sendRunComplete(runPeakHP, runPeakHP_RPM, runPeakTorque, runPeakTorque_RPM);
        recordingStartedMs = 0;
      }

      // ─── RECORD DATA POINTS ──────────────────────────────────────
      if (recording) {
        if (hp_smooth > runPeakHP && hp_smooth > 2.0f) {
          runPeakHP = hp_smooth;
          runPeakHP_RPM = leafFb.rpm;
        }
        if (torque_smooth > runPeakTorque && torque_smooth > 1.0f) {
          runPeakTorque = torque_smooth;
          runPeakTorque_RPM = leafFb.rpm;
        }

        static unsigned long lastRecordTime = 0;
        if (now - lastRecordTime >= 100) {
          if (leafFb.rpm > 400) {
            if (currentRun.size() >= MAX_RUN_POINTS)
              currentRun.erase(currentRun.begin());

            currentRun.push_back({ leafFb.rpm, hp_smooth, torque_smooth });
          }
          lastRecordTime = now;
        }
      }

      // ─── WEBSOCKET UPDATE (throttled to 20Hz) ────────────────────

      if (now - last_ws_update >= WS_UPDATE_MS) {
        // Both will work
        MetaSense::Notify::send(MetaSense::live, MetaSense::isRecording); // direct-no HAL
//        MetaSense::notifyClients(MetaSense::live, MetaSense::isRecording);// HAL

        last_ws_update = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(SENSOR_UPDATE_MS));
  }

}

// ─── SETUP ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

#if IS_ESP32S3
  while (!Serial && millis() < 3000) {
    delay(10);
  }
#endif

  canInit();

  rpmRamp.init(4000.0f);
  speedPI.init(0.05f, 0.10f, TORQUE_MIN, TORQUE_MAX);

  lastInvUpdateMs = millis();
  //---------------------------------------------------------------------------------------------
  Serial.println("\n\n╔══════════════════════════════════════════╗");
  Serial.println("║        MetaSense Dyno V" + String(FIRMWARE_VERSION) + "             ║");
  Serial.println("║        Build: " + String(BUILD_DATE) + "                   ║");
  Serial.println("╚══════════════════════════════════════════╝\n");
  //Patch 5
  pinMode(35, INPUT);                // Ramp function SW
  pinMode(36, INPUT);                // RB+ (12V contactor from VCU)
  pinMode(46, INPUT_PULLUP);         // BMH23M002 Data Ready intPin
  pinMode(38, OUTPUT);               // RB- FET-G
  pinMode(39, OUTPUT);               // SSSR

  ledcSetup(0, 50, 10);
  ledcAttachPin(45, 0);
  ledcSetup(1, 50, 10);
  ledcAttachPin(40, 1);
  ledcSetup(2, 50, 10);
  ledcAttachPin(41, 2);

  ledcWrite(0, 0);
  //myPID.setTimeStep(20);             // PID update interval 20ms
  //Patch 5 end

  delay(100);

  // Faster I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  validatePins();

  // Initialize LittleFS
  Serial.println("Starting LittleFS...");
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount FAILED! Formatting...");
    LittleFS.format();
    if (!LittleFS.begin()) {
      Serial.println("LittleFS still failed - continuing without filesystem");
    } else {
      Serial.println("LittleFS formatted and mounted");
    }
  } else {
    Serial.println("LittleFS mounted successfully");
  }
  // 1) ADC resolution
  adc1_config_width(ADC_WIDTH_BIT_12);

  // 2) ADC attenuation (0–3.3V)
  adc1_config_channel_atten(ADC_CH0, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(ADC_CH1, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(ADC_CH2, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(ADC_CH3, ADC_ATTEN_DB_12);
  adc1_config_channel_atten(ADC_CH4, ADC_ATTEN_DB_12);
  // 3) Calibration
  esp_adc_cal_characterize(
    ADC_UNIT_1,
    ADC_ATTEN_DB_12,
    ADC_WIDTH_BIT_12,
    1100,  // default Vref
    &adc1_chars);
  Serial.println("✅ ADC ready + calibrated");

  // Initialize BMP280
  Serial.println("Initializing BMP280...");
  bmp_initialized = bmp.begin(0x76);
  if (!bmp_initialized) {
    Serial.println("Trying BMP280 address 0x77...");
    bmp_initialized = bmp.begin(0x77);
  }
  if (bmp_initialized) {
    Serial.println("✅ BMP280 detected!");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  } else {
    Serial.println("⚠️  BMP280 not found - continuing without air density correction");
  }
  /*
  // Initialize AHT20
  Serial.println("Initializing AHT20...");
  aht_initialized = aht.begin();
  if (!aht_initialized) {
    Serial.println("Trying  address 0x39...");
    aht_initialized = aht.begin();
  }
  if (aht_initialized) Serial.println("✅ AHT20 detected!");
  else Serial.println("⚠️  AHT20 not found - continuing without RH correction");
*/
  // Initialize MCP9600 EGT sensor
  Serial.println("\nInitializing MCP9600 EGT sensor at 0x67...");
  mcp_egt_ok = mcp_egt.begin(MCP_EGT_ADDR);

  if (mcp_egt_ok) {
    mcp_egt.setThermocoupleType(MCP9600_TYPE_K);
    mcp_egt.setFilterCoefficient(3);
    mcp_egt.setADCresolution(MCP9600_ADCRESOLUTION_14);
    Serial.println("✅ MCP9600 EGT sensor initialized successfully at 0x67");

    // Test read to verify communication
    delay(100);
    float test_hot = mcp_egt.readThermocouple();
    float test_amb = mcp_egt.readAmbient();
    if (!isnan(test_hot) && !isnan(test_amb)) {
      Serial.printf("   Test reading: Hot=%.1f°C, Ambient=%.1f°C\n", test_hot, test_amb);
    } else {
      Serial.println("   Warning: Test read failed - check wiring");
    }
  } else {
    Serial.println("⚠️  MCP9600 EGT sensor NOT found at 0x67 - Check wiring / address");
  }

  // Initialize NAU7802
  Serial.println("Initializing NAU7802...");
// NAU7802 init
  nau_initialized = nau.begin(&Wire);
  if (!nau_initialized) {
    Serial.println("NAU7802 not found");
  } else {
    // Configure gain and sample rate as you had before
    nau.setGain(NAU7802_GAIN_128);          // example
    nau.setRate(NAU7802_RATE_80SPS);        // example
    // Optional: calibration etc.
  }


/* 
  nau_initialized = nau.begin();
  if (!nau_initialized) {
    Serial.println("NAU7802 not found!");
  } else {
    Serial.println("NAU7802 detected!");
    nau.setGain(NAU7802_GAIN_128);
    nau.setRate(NAU7802_RATE_320SPS);
  }
  */
  /*  
  // Set zero offset
  delay(500);
  long sum = 0;
  int count = 0;
  for (int i = 0; i < 16; i++) {
    if (nau_initialized && nau.available()) {
      sum += nau.read();
      count++;
      delay(5);
    }
  }
  if (count > 0) {
    zeroOffset = sum / count;
    Serial.print("Zero offset: ");
    Serial.println(zeroOffset);
  }
 */
  // Load preferences
  preferences.begin("dyno", false);
  CALIBRATION_FACTOR = preferences.getFloat("cal_factor", 0.025);
  LEVER_ARM_METERS = preferences.getFloat("lever_arm", 0.20);
  PULSES_PER_REVOLUTION = preferences.getFloat("pulses_per_rev", 2.0f);
  BRAKE_TO_ENGINE_RATIO = preferences.getFloat("brake_eng_ratio", 1.0f);
  RPM_FILTER_ALPHA = preferences.getFloat("rpm_filter", 0.25f);
  AUTO_HYSTERESIS = preferences.getFloat("auto_hyst", 0.0f);
  sta_ssid = preferences.getString("sta_ssid", "TP-Link_458C");
  sta_password = preferences.getString("sta_password", "metasense");
  //  sta_ssid               = preferences.getString("sta_ssid", "");
  //  sta_password           = preferences.getString("sta_password", "");
  DYNO_MODE = preferences.getString("dyno_mode", "brake");
  DRUM_MASS = preferences.getFloat("drum_mass", 10.0f);
  DRUM_DIAM_METERS = preferences.getFloat("drum_diam", 0.3f);
  PULSES_PER_REV_DRUM = preferences.getFloat("pulses_drum", 1.0f);
  DRUM_INERTIA_TYPE = preferences.getString("drum_inertia_type", "solid");
  DRUM_WALL_THICKNESS_CM = preferences.getFloat("drum_wall_cm", 2.0f);
  DRUM_INERTIA_CUSTOM = preferences.getFloat("drum_inertia_custom", 0.5f);
  RPM_SOURCE = preferences.getString("rpm_source", "spark");
  VIRTUAL_GEAR_RATIO = preferences.getFloat("virt_gear_ratio", 1.0f);
  //ENERGY = preferences.getFloat("h_reg[10]", 1.0f);
  DRIVETRAIN_EFFICIENCY = preferences.getFloat("drivetrain_eff", 1.0f);

  // WiFi setup
  if (sta_ssid.length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(sta_ssid.c_str(), sta_password.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) delay(500);
    if (WiFi.status() == WL_CONNECTED) {
#if IS_ESP32S3
      WiFi.setSleep(false);                 // no power save
      WiFi.setTxPower(WIFI_POWER_19_5dBm);  // max range
#endif
      Serial.println("Connected! IP: " + WiFi.localIP().toString());
    } else {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(ap_ssid, ap_password);
    }
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
  }
  Serial.println("AP IP: " + WiFi.softAPIP().toString());

  // DNS and server setup
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.addHandler(new CaptiveRequestHandler());

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "");
  });

  server.on("/version", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "MetaSense " + String(FIRMWARE_VERSION) + " - " + String(BUILD_DATE));
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/settings.html")) {
      request->send(LittleFS, "/settings.html", "text/html");
    } else {
      request->send(404, "text/plain", "settings.html not found");
    }
  });

  server.on("/trend.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/trend.html")) {
      request->send(LittleFS, "/trend.html", "text/html");
    } else {
      request->send(404, "text/plain", "trend.html not found");
    }
  });

  MetaSense::ws.onEvent(onWsEvent);
  server.addHandler(&MetaSense::ws);

  // OTA endpoints
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/update.html")) {
      request->send(LittleFS, "/update.html", "text/html");
    } else {
      request->send(404, "text/plain", "update.html not found");
    }
  });

  //Patch6
  /*
  // start the server
  wifiserver.begin();
  if (!modbusTCPServer.begin()) {
    Serial.println("ModbusServer not running");
  }
  Serial.println("ModbusServer running");
  modbusTCPServer.configureCoils(0x00, 1);
  modbusTCPServer.configureHoldingRegisters(0x00, 15);
  printWiFiStatus();
*/
  //Patch6 end
  server.on(
    "/update", HTTP_POST, [](AsyncWebServerRequest *request) {
      String msg = Update.hasError() ? "FAIL" : "OK - Rebooting...";
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", msg);
      response->addHeader("Connection", "close");
      request->send(response);
      if (!Update.hasError()) {
        delay(1000);
        ESP.restart();
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!index) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      }
      if (Update.write(data, len) != len) Update.printError(Serial);
      if (final) {
        if (Update.end(true)) Serial.println("OTA Success");
        else Update.printError(Serial);
      }
    });

  server.on("/update_fs", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/update_fs.html")) {
      request->send(LittleFS, "/update_fs.html", "text/html");
    } else {
      request->send(404, "text/plain", "update_fs.html not found");
    }
  });

  server.on(
    "/update_fs", HTTP_POST, [](AsyncWebServerRequest *request) {
      String msg = Update.hasError() ? "Filesystem update FAILED!" : "Filesystem updated!";
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", msg);
      response->addHeader("Connection", "close");
      request->send(response);
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      static bool fsUpdateStarted = false;
      if (!index) {
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {
          Update.printError(Serial);
          fsUpdateStarted = false;
          return;
        }
        fsUpdateStarted = true;
      }
      if (fsUpdateStarted && len > 0) Update.write(data, len);
      if (final && fsUpdateStarted) {
        if (Update.end(true)) Serial.println("FS Update Success!");
        else Update.printError(Serial);
        fsUpdateStarted = false;
      }
    });

  server.begin();

  // Setup interrupts
  pinMode(HALL_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
  pinMode(DRUM_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(DRUM_PIN), drumISR, FALLING);

  // Initialize median buffers
  memset(rpmMedian, 0, sizeof(rpmMedian));
  memset(drumMedian, 0, sizeof(drumMedian));

  broadcastSettings();

  // Create tasks
  xTaskCreatePinnedToCore(calibrationTask, "CalibTask", 4096, NULL, 1, &calibTask, 1);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 2, NULL, 0);

  Serial.println("\n✅ Setup complete - MetaSense v 1.2");
  Serial.println("   Dashboard: http://" + WiFi.softAPIP().toString());
  Serial.printf("   Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.println("   EGT reading at 15Hz with retry logic");
}

// ─── LOOP ────────────────────────────────────────────────────────
void loop() {
  dnsServer.processNextRequest();
  MetaSense::ws.cleanupClients();
 /*
  //Patch7
  // listen for incoming clients
  WiFiClient client = wifiserver.available();
  if (client) {
    // a new client connected
    Serial.println("new client");

    // let the Modbus TCP accept the connection
    modbusTCPServer.accept(client);

    while (client.connected()) {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
      for (int j = 0; j < 15; j++) {
        modbusTCPServer.holdingRegisterWrite(j, h_reg[j]);
      }
          
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint >= 65000) {
          lastPrint = millis();
          Serial.printf("Client connected ✅ Heap: %d bytes | Runs: %d | EGT: %.1f°C\n",
          ESP.getFreeHeap(), (int)savedRuns.size(), egtTracker.getFiltered());
        }

    }
    Serial.println("client disconnected");
  }
*/
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 65000) {
    lastPrint = millis();
    Serial.printf("✅ Heap: %d bytes | Runs: %d | EGT: %.1f°C\n",
                  ESP.getFreeHeap(), (int)savedRuns.size(), egtTracker.getFiltered());
  }

  vTaskDelay(pdMS_TO_TICKS(1));

}

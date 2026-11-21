#pragma once
#include <Arduino.h>
#include <Wire.h>

// NAU7802 I2C address
#define NAU7802_I2C_ADDR 0x2A

// Register map (subset)
#define NAU7802_PU_CTRL   0x00
#define NAU7802_CTRL1     0x01
#define NAU7802_CTRL2     0x02
#define NAU7802_ADCO_B2   0x12
#define NAU7802_ADCO_B1   0x13
#define NAU7802_ADCO_B0   0x14
#define NAU7802_INT_STAT  0x02  // DRDY bit also readable here on some revs

// PU_CTRL register bits (NAU7802 datasheet, section 9.3)
#define NAU7802_PU_CTRL_RR    0x01  // Register Reset (write 1 to reset)
#define NAU7802_PU_CTRL_PUD   0x02  // Power Up Digital circuit
#define NAU7802_PU_CTRL_PUA   0x04  // Power Up Analog circuit
#define NAU7802_PU_CTRL_PUR   0x08  // Power Up Ready (read-only)
#define NAU7802_PU_CTRL_CS    0x10  // Cycle Start (start conversion)
#define NAU7802_PU_CTRL_CR    0x20  // Conversion Ready (read-only)
#define NAU7802_PU_CTRL_OSCS  0x40  // System clock source select
#define NAU7802_PU_CTRL_AVDDS 0x80  // AVDD source select

// CTRL2 calibration-related bits
#define NAU7802_CTRL2_CALMOD_MASK 0x03
#define NAU7802_CTRL2_CALS        0x04
#define NAU7802_CTRL2_CAL_ERROR   0x08

// Gain enum
typedef enum _gains {
  NAU7802_GAIN_1  = 0,
  NAU7802_GAIN_2  = 1,
  NAU7802_GAIN_4  = 2,
  NAU7802_GAIN_8  = 3,
  NAU7802_GAIN_16 = 4,
  NAU7802_GAIN_32 = 5,
  NAU7802_GAIN_64 = 6,
  NAU7802_GAIN_128 = 7
} NAU7802_Gain;

// Sample rate enum
typedef enum _sample_rates {
  NAU7802_RATE_10SPS  = 0,
  NAU7802_RATE_20SPS  = 1,
  NAU7802_RATE_40SPS  = 2,
  NAU7802_RATE_80SPS  = 3,
  NAU7802_RATE_320SPS = 7
} NAU7802_SampleRate;

// Internal LDO output voltage select (CTRL1 bits [5:3])
typedef enum _ldo_values {
  NAU7802_LDO_2V4 = 7,
  NAU7802_LDO_2V7 = 6,
  NAU7802_LDO_3V0 = 5,
  NAU7802_LDO_3V3 = 4,
  NAU7802_LDO_3V6 = 3,
  NAU7802_LDO_3V9 = 2,
  NAU7802_LDO_4V2 = 1,
  NAU7802_LDO_4V5 = 0
} NAU7802_LDO;

// Calibration mode
typedef enum _calib_mode {
  NAU7802_CALMOD_INTERNAL = 0,
  NAU7802_CALMOD_OFFSET   = 2,
  NAU7802_CALMOD_GAIN     = 3
} NAU7802_Calibration;

class Adafruit_NAU7802 {
public:
  Adafruit_NAU7802();

  bool begin(TwoWire *wire = &Wire);

  bool available();
  int32_t read();

  void setGain(NAU7802_Gain gain);
  void setRate(NAU7802_SampleRate rate);
  bool setLDO(NAU7802_LDO ldo);
  bool calibrate(NAU7802_Calibration mode, uint16_t timeout_ms = 1000);
  bool resetAndPowerUp();

  // Optional: expose raw register access if you want later
  uint8_t readRegister(uint8_t reg, bool *ok = nullptr);
  bool writeRegister(uint8_t reg, uint8_t value);

private:
  TwoWire *_wire;
  bool _initialized;

  bool reset();
  bool powerUp();
  bool waitForPowerUp(uint16_t timeout_ms = 100);
  bool waitForConversion(uint16_t timeout_ms = 50);
};

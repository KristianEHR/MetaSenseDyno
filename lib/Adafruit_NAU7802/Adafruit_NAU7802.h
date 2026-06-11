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

// Bits
#define NAU7802_PU_CTRL_PUD   0x01
#define NAU7802_PU_CTRL_PUA   0x02
#define NAU7802_PU_CTRL_PUR   0x04
#define NAU7802_PU_CTRL_CS    0x08
#define NAU7802_PU_CTRL_CR    0x10
#define NAU7802_PU_CTRL_OSCS  0x20
#define NAU7802_PU_CTRL_AVDDS 0x40

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

// Calibration mode (stubbed for now)
typedef enum _calib_mode {
  NAU7802_CALIB_OFFSET = 0,
  NAU7802_CALIB_GAIN   = 1
} NAU7802_Calibration;

class Adafruit_NAU7802 {
public:
  Adafruit_NAU7802();

  bool begin(TwoWire *wire = &Wire);

  bool available();
  int32_t read();

  void setGain(NAU7802_Gain gain);
  void setRate(NAU7802_SampleRate rate);

  // Optional: expose raw register access if you want later
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

private:
  TwoWire *_wire;
  bool _initialized;

  bool reset();
  bool powerUp();
  bool waitForPowerUp(uint16_t timeout_ms = 100);
  bool waitForConversion(uint16_t timeout_ms = 50);
};

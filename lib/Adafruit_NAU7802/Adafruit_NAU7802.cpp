#include "Adafruit_NAU7802.h"

Adafruit_NAU7802::Adafruit_NAU7802()
: _wire(&Wire), _initialized(false) {}

bool Adafruit_NAU7802::begin(TwoWire *wire) {
  if (wire == nullptr) {
    return false;
  }
  _wire = wire;
  // Do NOT call _wire->begin() — the application configures I2C pins/clock.

  // Step 1: assert register reset (RR = bit 0)
  writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR);
  delay(2);

  // Step 2: deassert reset, power up digital + analog circuits
  writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);

  // Step 3: wait for PUR (Power Up Ready, bit 3) to indicate both supplies ready
  uint32_t start = millis();
  while (millis() - start < 200) {
    uint8_t pu = readRegister(NAU7802_PU_CTRL);
    if (pu & NAU7802_PU_CTRL_PUR) {
      _initialized = true;
      return true;
    }
    delay(5);
  }
  return false;
}

bool Adafruit_NAU7802::reset() {
  writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR);
  delay(2);
  return true;
}

bool Adafruit_NAU7802::powerUp() {
  writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
  return waitForPowerUp();
}

bool Adafruit_NAU7802::waitForPowerUp(uint16_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t pu = readRegister(NAU7802_PU_CTRL);
    if (pu & NAU7802_PU_CTRL_PUR) {
      return true;
    }
    delay(5);
  }
  return false;
}

void Adafruit_NAU7802::setGain(NAU7802_Gain gain) {
  uint8_t ctrl1 = readRegister(NAU7802_CTRL1);
  ctrl1 &= ~0x07;          // clear gain bits
  ctrl1 |= (uint8_t)gain;  // set new gain
  writeRegister(NAU7802_CTRL1, ctrl1);
}

void Adafruit_NAU7802::setRate(NAU7802_SampleRate rate) {
  uint8_t ctrl2 = readRegister(NAU7802_CTRL2);
  ctrl2 &= ~0x07;           // clear rate bits
  ctrl2 |= (uint8_t)rate;   // set new rate
  writeRegister(NAU7802_CTRL2, ctrl2);
}

bool Adafruit_NAU7802::available() {
  if (!_initialized) return false;
  // CR (Conversion Ready) is bit 5 of PU_CTRL
  uint8_t pu = readRegister(NAU7802_PU_CTRL);
  return (pu & NAU7802_PU_CTRL_CR) != 0;
}

bool Adafruit_NAU7802::waitForConversion(uint16_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (available()) return true;
    delay(1);
  }
  return false;
}

int32_t Adafruit_NAU7802::read() {
  if (!waitForConversion()) {
    return 0;
  }

  uint8_t b2 = readRegister(NAU7802_ADCO_B2);
  uint8_t b1 = readRegister(NAU7802_ADCO_B1);
  uint8_t b0 = readRegister(NAU7802_ADCO_B0);

  int32_t value = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  // Sign-extend 24-bit to 32-bit
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return value;
}

uint8_t Adafruit_NAU7802::readRegister(uint8_t reg) {
  _wire->beginTransmission(NAU7802_I2C_ADDR);
  _wire->write(reg);
  _wire->endTransmission(false);

  _wire->requestFrom((int)NAU7802_I2C_ADDR, 1);
  if (_wire->available()) {
    return _wire->read();
  }
  return 0;
}

void Adafruit_NAU7802::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(NAU7802_I2C_ADDR);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

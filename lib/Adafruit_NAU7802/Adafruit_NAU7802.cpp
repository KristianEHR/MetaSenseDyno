#include "Adafruit_NAU7802.h"

Adafruit_NAU7802::Adafruit_NAU7802()
: _wire(&Wire), _initialized(false) {}

bool Adafruit_NAU7802::begin(TwoWire *wire) {
  _wire = wire;
  _wire->begin();

  if (!reset()) {
    return false;
  }
  if (!powerUp()) {
    return false;
  }
  _initialized = true;
  return true;
}

bool Adafruit_NAU7802::reset() {
  // Simple reset: toggle PU_CTRL bits
  writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
  delay(2);
  writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA | NAU7802_PU_CTRL_PUR);
  delay(2);
  return true;
}

bool Adafruit_NAU7802::powerUp() {
  // Enable analog + digital + oscillator
  uint8_t pu = NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA | NAU7802_PU_CTRL_OSCS | NAU7802_PU_CTRL_AVDDS;
  writeRegister(NAU7802_PU_CTRL, pu);
  return waitForPowerUp();
}

bool Adafruit_NAU7802::waitForPowerUp(uint16_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t pu = readRegister(NAU7802_PU_CTRL);
    if (pu & NAU7802_PU_CTRL_PUA) {
      return true;
    }
    delay(2);
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
  // DRDY is bit 5 in CTRL2 or INT_STAT depending on revision; we poll INT_STAT here
  uint8_t stat = readRegister(NAU7802_INT_STAT);
  return (stat & 0x20) != 0; // DRDY bit
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

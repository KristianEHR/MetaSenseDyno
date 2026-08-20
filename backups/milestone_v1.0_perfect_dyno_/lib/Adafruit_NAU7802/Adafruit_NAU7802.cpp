#include "Adafruit_NAU7802.h"

Adafruit_NAU7802::Adafruit_NAU7802()
: _wire(&Wire), _initialized(false) {}

bool Adafruit_NAU7802::begin(TwoWire *wire) {
  if (wire == nullptr) {
    return false;
  }
  _wire = wire;
  _initialized = false;
  // Do NOT call _wire->begin() — the application configures I2C pins/clock.

  // Ensure device responds before attempting register writes.
  _wire->beginTransmission(NAU7802_I2C_ADDR);
  if (_wire->endTransmission() != 0) {
    return false;
  }

  // Step 1: assert register reset (RR = bit 0)
  if (!writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR)) {
    return false;
  }
  delay(2);

  // Step 2: deassert reset, power up digital + analog circuits
  // Also assert CS so conversion cycle starts deterministically.
  if (!writeRegister(NAU7802_PU_CTRL,
                     NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA | NAU7802_PU_CTRL_CS)) {
    return false;
  }

  // Step 3: wait for PUR (Power Up Ready, bit 3) to indicate both supplies ready
  uint32_t start = millis();
  while (millis() - start < 200) {
    bool ok = false;
    uint8_t pu = readRegister(NAU7802_PU_CTRL, &ok);
    if (ok && (pu & NAU7802_PU_CTRL_PUR)) {
      _initialized = true;
      return true;
    }
    delay(5);
  }
  return false;
}

bool Adafruit_NAU7802::reset() {
  if (!writeRegister(NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR)) {
    return false;
  }
  delay(2);
  return true;
}

bool Adafruit_NAU7802::powerUp() {
  if (!writeRegister(NAU7802_PU_CTRL,
                     NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA | NAU7802_PU_CTRL_CS)) {
    return false;
  }
  return waitForPowerUp();
}

bool Adafruit_NAU7802::waitForPowerUp(uint16_t timeout_ms) {
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    bool ok = false;
    uint8_t pu = readRegister(NAU7802_PU_CTRL, &ok);
    if (ok && (pu & NAU7802_PU_CTRL_PUR)) {
      return true;
    }
    delay(5);
  }
  return false;
}

void Adafruit_NAU7802::setGain(NAU7802_Gain gain) {
  bool ok = false;
  uint8_t ctrl1 = readRegister(NAU7802_CTRL1, &ok);
  if (!ok) {
    return;
  }
  ctrl1 &= ~0x07;          // clear gain bits
  ctrl1 |= (uint8_t)gain;  // set new gain
  (void)writeRegister(NAU7802_CTRL1, ctrl1);
}

void Adafruit_NAU7802::setRate(NAU7802_SampleRate rate) {
  bool ok = false;
  uint8_t ctrl2 = readRegister(NAU7802_CTRL2, &ok);
  if (!ok) {
    return;
  }
  // CRS (sample-rate select) is CTRL2 bits [6:4].
  ctrl2 &= static_cast<uint8_t>(~0x70);
  ctrl2 |= static_cast<uint8_t>((static_cast<uint8_t>(rate) & 0x07) << 4);
  (void)writeRegister(NAU7802_CTRL2, ctrl2);
}

bool Adafruit_NAU7802::setLDO(NAU7802_LDO ldo) {
  bool ok = false;
  uint8_t ctrl1 = readRegister(NAU7802_CTRL1, &ok);
  if (!ok) {
    return false;
  }

  // CTRL1 bits [5:3] select internal LDO voltage.
  ctrl1 &= static_cast<uint8_t>(~0x38);
  ctrl1 |= static_cast<uint8_t>((static_cast<uint8_t>(ldo) & 0x07) << 3);
  if (!writeRegister(NAU7802_CTRL1, ctrl1)) {
    return false;
  }

  // Enable internal AVDD LDO source.
  bool puOk = false;
  uint8_t puCtrl = readRegister(NAU7802_PU_CTRL, &puOk);
  if (!puOk) {
    return false;
  }
  puCtrl |= NAU7802_PU_CTRL_AVDDS;
  return writeRegister(NAU7802_PU_CTRL, puCtrl);
}

bool Adafruit_NAU7802::resetAndPowerUp() {
  if (!_initialized) {
    return false;
  }
  if (!reset()) {
    return false;
  }
  return powerUp();
}

bool Adafruit_NAU7802::calibrate(NAU7802_Calibration mode, uint16_t timeout_ms) {
  if (!_initialized) {
    return false;
  }

  bool ok = false;
  uint8_t ctrl2 = readRegister(NAU7802_CTRL2, &ok);
  if (!ok) {
    return false;
  }

  // Preserve sample-rate bits and set calibration mode/start bits.
  ctrl2 &= static_cast<uint8_t>(~(NAU7802_CTRL2_CALMOD_MASK | NAU7802_CTRL2_CALS | NAU7802_CTRL2_CAL_ERROR));
  ctrl2 |= static_cast<uint8_t>(static_cast<uint8_t>(mode) & NAU7802_CTRL2_CALMOD_MASK);
  ctrl2 |= NAU7802_CTRL2_CALS;
  if (!writeRegister(NAU7802_CTRL2, ctrl2)) {
    return false;
  }

  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t status = readRegister(NAU7802_CTRL2, &ok);
    if (!ok) {
      return false;
    }

    if ((status & NAU7802_CTRL2_CAL_ERROR) != 0) {
      return false;
    }

    if ((status & NAU7802_CTRL2_CALS) == 0) {
      return true;
    }

    delay(2);
  }

  return false;
}

bool Adafruit_NAU7802::available() {
  if (!_initialized) return false;
  // CR (Conversion Ready) is bit 5 of PU_CTRL
  bool ok = false;
  uint8_t pu = readRegister(NAU7802_PU_CTRL, &ok);
  return ok && ((pu & NAU7802_PU_CTRL_CR) != 0);
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

  bool ok2 = false;
  bool ok1 = false;
  bool ok0 = false;
  uint8_t b2 = readRegister(NAU7802_ADCO_B2, &ok2);
  uint8_t b1 = readRegister(NAU7802_ADCO_B1, &ok1);
  uint8_t b0 = readRegister(NAU7802_ADCO_B0, &ok0);
  if (!(ok2 && ok1 && ok0)) {
    return 0;
  }

  int32_t value = ((int32_t)b2 << 16) | ((int32_t)b1 << 8) | b0;
  // Sign-extend 24-bit to 32-bit
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return value;
}

uint8_t Adafruit_NAU7802::readRegister(uint8_t reg, bool *ok) {
  if (ok) {
    *ok = false;
  }
  _wire->beginTransmission(NAU7802_I2C_ADDR);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return 0;
  }

  if (_wire->requestFrom((int)NAU7802_I2C_ADDR, 1) != 1) {
    return 0;
  }
  if (_wire->available()) {
    if (ok) {
      *ok = true;
    }
    return _wire->read();
  }
  return 0;
}

bool Adafruit_NAU7802::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(NAU7802_I2C_ADDR);
  _wire->write(reg);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

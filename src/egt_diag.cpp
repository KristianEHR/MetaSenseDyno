#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP9600.h>

namespace {

constexpr uint8_t kSdaPin = 17;
constexpr uint8_t kSclPin = 18;
constexpr uint32_t kI2cClockHz = 50000;
constexpr uint16_t kI2cTimeoutMs = 50;
constexpr uint16_t kInitSettleDelayMs = 300;
constexpr uint8_t kPreferredAddr = 0x67;
constexpr uint8_t kMinProbeAddr = 0x60;
constexpr uint8_t kMaxProbeAddr = 0x67;

Adafruit_MCP9600 mcp;
bool mcpReady = false;
uint8_t mcpAddr = 0;
uint32_t lastReadMs = 0;

void logLine(const char* msg) {
  Serial.println(msg);
  Serial0.println(msg);
}

void logf(const char* fmt, ...) {
  char buffer[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  Serial.print(buffer);

  Serial0.print(buffer);
}

uint8_t i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission();
}

const char* i2cErrToText(uint8_t err) {
  switch (err) {
    case 0: return "ACK";
    case 1: return "DATA_TOO_LONG";
    case 2: return "NACK_ADDR";
    case 3: return "NACK_DATA";
    case 4: return "OTHER";
    case 5: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}

void scanI2c() {
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    const uint8_t err = i2cProbe(addr);
    if (err == 0) {
      ++found;
      logf("[EGT-DIAG] I2C ACK @ 0x%02X\n", addr);
    }
  }
  logf("[EGT-DIAG] I2C scan complete: %u device(s)\n", found);
}

bool tryInitAt(uint8_t addr) {
  const uint8_t err = i2cProbe(addr);
  logf("[EGT-DIAG] probe 0x%02X err=%u (%s)\n", addr, err, i2cErrToText(err));
  if (err != 0) {
    return false;
  }

  if (!mcp.begin(addr, &Wire)) {
    logf("[EGT-DIAG] mcp.begin failed at 0x%02X\n", addr);
    return false;
  }

  mcp.setThermocoupleType(MCP9600_TYPE_K);
  mcp.setFilterCoefficient(3);
  mcp.setADCresolution(MCP9600_ADCRESOLUTION_14);
  mcp.setAmbientResolution(RES_ZERO_POINT_0625);
  mcp.enable(true);
  delay(20);

  const float hot = mcp.readThermocouple();
  const float amb = mcp.readAmbient();
  const uint8_t status = mcp.getStatus();
  logf("[EGT-DIAG] init OK @0x%02X status=0x%02X hot=%.2f amb=%.2f\n", addr, status, hot, amb);

  if (!isfinite(hot) || !isfinite(amb)) {
    logLine("[EGT-DIAG] init read invalid, treating as not ready");
    return false;
  }

  mcpReady = true;
  mcpAddr = addr;
  return true;
}

void initMcp() {
  mcpReady = false;
  mcpAddr = 0;

  if (tryInitAt(kPreferredAddr)) {
    return;
  }

  for (uint8_t addr = kMinProbeAddr; addr <= kMaxProbeAddr; ++addr) {
    if (addr == kPreferredAddr) {
      continue;
    }
    if (tryInitAt(addr)) {
      return;
    }
  }

  logLine("[EGT-DIAG] MCP9600 not detected");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);
  delay(200);

  logLine("[EGT-DIAG] boot");
  logf("[EGT-DIAG] I2C pins SDA=%u SCL=%u\n", kSdaPin, kSclPin);

  Wire.begin(kSdaPin, kSclPin);
  Wire.setClock(kI2cClockHz);
  Wire.setTimeOut(kI2cTimeoutMs);
  logf("[EGT-DIAG] I2C clock=%lu timeout=%u\n",
       static_cast<unsigned long>(kI2cClockHz),
       static_cast<unsigned>(kI2cTimeoutMs));

  scanI2c();
  delay(kInitSettleDelayMs);
  initMcp();
}

void loop() {
  const uint32_t now = millis();
  if ((now - lastReadMs) < 2000) {
    delay(20);
    return;
  }
  lastReadMs = now;

  if (!mcpReady) {
    logLine("[EGT-DIAG] retrying MCP9600 init...");
    initMcp();
    return;
  }

  const uint8_t status = mcp.getStatus();
  const float hot = mcp.readThermocouple();
  const float amb = mcp.readAmbient();

  if (!isfinite(hot) || !isfinite(amb)) {
    logf("[EGT-DIAG] read invalid @0x%02X status=0x%02X, will re-init\n", mcpAddr, status);
    mcpReady = false;
    return;
  }

  logf("[EGT-DIAG] addr=0x%02X status=0x%02X hot=%.2f amb=%.2f\n", mcpAddr, status, hot, amb);
}

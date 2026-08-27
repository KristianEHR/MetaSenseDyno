#pragma once
#include <Arduino.h>

namespace MetaSense::Input {

    void tareMainGui();
    void tare();
    bool requestTare();
    void setCalibrationFactor(float factor);
    bool calibrateWithKnownWeight(float knownWeightKg, float& outFactor);
    bool requestCalibrationWithKnownWeight(float knownWeightKg);
    float getCalibrationFactor();
    float getZeroOffset();
    float getZeroOffsetRaw();
    uint16_t getLoadCellSampleRateSps();
    bool applyLoadCellSettingsProfile();

    // Telemetry accessors
    float rpm();
    float torqueNm();
    float egtHotC();
    void getEnvironment(float& ambientC,
                        float& pressureHpa,
                        float& humidityPct,
                        float& airDensity,
                        float& climateCf);
    float currentKpLive();
    bool isVcuReady();
    const char* vcuReadySource();
    void setUiModeHintTrend(bool trendMode);
    bool isUiModeHintTrend();
    void getLoadCellInitStatus(bool& ldoConfigured, bool& internalCalOk, uint8_t& internalCalAttempts);
    void getLoadCellSamplerRuntime(uint32_t& lastUs, uint32_t& maxUs, uint32_t& emaUs, uint32_t& loops);
    void resetLoadCellSamplerMaxRuntime();

    // Update CAN-derived engine RPM input.
    void updateCanRpm(float rpm);
    void updateCanTorque(float torqueNm);
    void updateCanTemps(float inverterTempC, float statorTempC, float coolantTempC);
    void updateCanStatus(bool ready, bool fault, bool warning, bool limp);
    void setLeafUiTorqueDemandNm(float torqueNm);
    float getLeafUiTorqueDemandNm();
    void setLeafManualTorqueMode(bool manualMode);
    bool getLeafManualTorqueMode();
    void setLeaf11aUiGear(uint8_t gear);
    uint8_t getLeaf11aUiGear();
    void setLeaf11aUiCarOnOff(uint8_t carOnOff);
    uint8_t getLeaf11aUiCarOnOff();
    bool sendLeafTorqueCommand1d4(float torqueDemandNm,
                                  bool inverterReadyBit,
                                  bool hvOkBit,
                                  bool brakeBit,
                                  bool gearDriveBit);
    bool sendLeafTorqueCommand1d4FinalZero();
    void updateVcuDebug(bool simMode,
                        bool inv12v,
                        float hvVoltage,
                        float torqueDemandNm,
                        bool rPlus,
                        bool precharge,
                        bool ssr,
                        bool rMinus);

    // Initialize the dyno input/control subsystem
    void begin();

    // Main dyno control loop (called from loop())
    void loop();

    // Network-side telemetry publish loop.
    void publish();

    // Start a dyno recording session (resets peaks, integrators, etc.)
    void startRecording();

    // Stop a dyno recording session (zero throttle/brake)
    void stopRecording();

} // namespace MetaSense::Input

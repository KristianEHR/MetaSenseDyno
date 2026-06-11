#pragma once
namespace MetaSense::Input {

    void tare();
    void setCalibrationFactor(float factor);
    float getCalibrationFactor();
    float getZeroOffset();

    // Telemetry accessors
    float rpm();
    float torqueNm();

    // Update CAN-derived engine RPM input.
    void updateCanRpm(float rpm);

    // Initialize the dyno input/control subsystem
    void begin();

    // Main dyno control loop (called from loop())
    void loop();

    // Start a dyno recording session (resets peaks, integrators, etc.)
    void startRecording();

    // Stop a dyno recording session (zero throttle/brake)
    void stopRecording();

} // namespace MetaSense::Input

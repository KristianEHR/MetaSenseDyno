#include "DynoStateMachine.h"

#include "Input.h"
#include "Inverter.h"
#include "Settings.h"
#include "Notify.h"

#include <Arduino.h>

#include <math.h>

namespace MetaSense::WebSocketServer {
void sendStatus(const String& msg);
}

namespace MetaSense::HardwareOutputStateMachine {
void stop();
}

namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Internal state
// ─────────────────────────────────────────────────────────────────────────────

enum class State {
    MANUAL,
    AUTO_IDLE,
    AUTO_UP_LINEAR,
    AUTO_UP_EXP,
    AUTO_DOWN_EXP,
    AUTO_TIMEOUT_DOWN,
    AUTO_DONE,
    AUTO_ABORT
};

State state = State::MANUAL;

// GUI + panel mode flags
bool autoMode   = false;   // GUI Auto Run button
bool panelAuto  = false;   // Panel switch (AUTO position)
uint32_t autoRunStartMs = 0;
bool safetyShutdownActive = false;
bool restartRequired = false;

// Recording latch (kept from your original design)
bool recording  = false;

// Manual RPM target (from panel pot, already filtered/slew-limited upstream)
float manualRpmTarget = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
// Exponential ramp helper
// ─────────────────────────────────────────────────────────────────────────────

struct ExpRamp {
    float value  = 0.0f;
    float target = 0.0f;
    float alpha  = 0.0f;

    void init(float start, float tgt, float tau, float dt) {
        value  = start;
        target = tgt;
        alpha  = 1.0f - expf(-dt / tau);
    }

    float update() {
        value += alpha * (target - value);
        return value;
    }
};

ExpRamp rpmRampUp;
ExpRamp rpmRampDown;

// ─────────────────────────────────────────────────────────────────────────────
// Timing and sweep parameters
// ─────────────────────────────────────────────────────────────────────────────

constexpr float dtSeconds   = 0.02f;   // 20 ms loop
constexpr float tauUp       = 5.0f;    // exponential up-sweep time constant
constexpr float tauDown     = 5.0f / 4.0f; // 4× faster down-slope

constexpr float linearRate  = 200.0f;  // RPM/s for initial linear phase
constexpr uint32_t autoRunTimeoutMs = 25000;
constexpr float safetyShutdownTargetRpm = 1500.0f;

// Measurement window (old behavior: 7000 → 14000 RPM)
constexpr float measureRpmMin = 7000.0f;
constexpr float measureRpmMax = 14000.0f;

// ─────────────────────────────────────────────────────────────────────────────
// Energy (MJ) accumulator
// ─────────────────────────────────────────────────────────────────────────────

float energyMJ      = 0.0f;
bool  measuringMJ   = false;
float lastRunEnergy = 0.0f;
bool  runFinished   = false;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

inline float computePowerKW(float rpm, float torqueNm)
{
    // P = 2π * rpm/60 * torque [W] → /1000 → kW
    const float omega = rpm * (2.0f * 3.14159265359f / 60.0f);
    return (omega * torqueNm) / 1000.0f;
}

inline void resetEnergy()
{
    energyMJ    = 0.0f;
    measuringMJ = false;
}

inline void startMeasuringIfInWindow(float rpm)
{
    if (rpm >= measureRpmMin && rpm <= measureRpmMax) {
        measuringMJ = true;
    }
}

inline void updateEnergy(float rpm, float torqueNm)
{
    if (!measuringMJ) {
        return;
    }

    if (rpm < measureRpmMin || rpm > measureRpmMax) {
        measuringMJ = false;
        return;
    }

    float powerKW = computePowerKW(rpm, torqueNm);
    // kW * s = kJ → /1000 = MJ
    energyMJ += (powerKW * dtSeconds) / 1000.0f;
}

inline void beginSafetyShutdown(float rpm)
{
    safetyShutdownActive = true;
    autoRunStartMs = 0;
    rpmRampDown.init(rpm, safetyShutdownTargetRpm, tauDown, dtSeconds);
    MetaSense::DynoStateMachine::setTorqueFeedForward(0.0f);
    MetaSense::Settings::setRpmTarget(rpm);
    MetaSense::WebSocketServer::sendStatus("Auto run timeout: ramping down to 1500 RPM");
}

inline void completeSafetyShutdown()
{
    if (recording) {
        MetaSense::DynoStateMachine::stopRecording();
    }

    lastRunEnergy = energyMJ;
    runFinished   = true;
    autoMode      = false;
    panelAuto     = false;
    safetyShutdownActive = false;
    restartRequired = true;
    state = State::MANUAL;

    MetaSense::DynoStateMachine::setTorqueFeedForward(0.0f);
    MetaSense::Settings::setRpmTarget(0.0f);
    MetaSense::HardwareOutputStateMachine::stop();
    MetaSense::WebSocketServer::sendStatus("System restart required");
}

} // anonymous namespace


namespace MetaSense::DynoStateMachine {

// ─────────────────────────────────────────────────────────────────────────────
// Recording API (kept and extended)
// ─────────────────────────────────────────────────────────────────────────────

void startRecording()
{
    recording = true;
    MetaSense::Input::startRecording();
}

void stopRecording()
{
    recording = false;
    MetaSense::DynoStateMachine::setTorqueFeedForward(0.0f);
    MetaSense::Input::stopRecording();
}

bool isRecording()
{
    return recording;
}

bool isAutoRunActive()
{
    const bool sw = (panelAuto || recording);
    return autoMode && sw;
}

bool isSafetyShutdownActive()
{
    return safetyShutdownActive;
}

bool isRestartRequired()
{
    return restartRequired;
}

void setTorqueFeedForward(float torque)
{
    MetaSense::Inverter::setTorqueFeedForward(torque);
}

float torqueFeedForward()
{
    return MetaSense::Inverter::torqueFeedForward();
}

// ─────────────────────────────────────────────────────────────────────────────
// External control inputs (GUI + panel)
// ─────────────────────────────────────────────────────────────────────────────

void setPanelAuto(bool enabled)
{
    panelAuto = enabled;
    // When panel is forced to MANUAL and we are not recording, reset energy
    if (!panelAuto && !recording) {
        resetEnergy();
        state = State::MANUAL;
    }
}

void setAutoMode(bool enabled)
{
    if (enabled && restartRequired) {
        MetaSense::WebSocketServer::sendStatus("System restart required before a new run");
        return;
    }

    autoMode = enabled;
    if (autoMode) {
        autoRunStartMs = millis();
        runFinished = false;
        state = State::AUTO_IDLE;
        return;
    }

    if (!autoMode) {
        // Leaving auto mode → go back to manual behavior
        if (!recording) {
            resetEnergy();
        }
        state = State::MANUAL;
    }
}

void setManualRpmTarget(float rpm)
{
    manualRpmTarget = rpm;
}

void abortAutoRun()
{
    if (safetyShutdownActive || restartRequired) {
        return;
    }

    // Abort: stop recording, reset energy, go back to manual
    if (recording) {
        stopRecording();
    }
    autoMode = false;
    resetEnergy();
    runFinished = false;
    state = State::MANUAL;
}

// ─────────────────────────────────────────────────────────────────────────────
// Run result access
// ─────────────────────────────────────────────────────────────────────────────

float lastRunEnergyMJ()
{
    return lastRunEnergy;
}

bool isRunFinished()
{
    return runFinished;
}

void clearRunFinished()
{
    runFinished = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main update loop (call every 20 ms)
// ─────────────────────────────────────────────────────────────────────────────

void update()
{
    // Inputs
    float rpm       = MetaSense::Input::rpm();
    float torqueNm  = MetaSense::Input::torqueNm();

    // GUI-configured sweep limits
    float rpmStart  = MetaSense::Settings::rpmStart();
    float rpmEnd    = MetaSense::Settings::rpmEnd();

    // Old behavior: sw = panelAuto || recording
    bool sw = (panelAuto || recording);

    if (restartRequired) {
        MetaSense::Settings::setRpmTarget(0.0f);
        return;
    }

    if (safetyShutdownActive) {
        float commanded = rpmRampDown.update();
        MetaSense::Settings::setRpmTarget(commanded);
        setTorqueFeedForward(0.0f);

        if (commanded <= safetyShutdownTargetRpm * 1.001f) {
            completeSafetyShutdown();
        }

        return;
    }

    // If not in auto mode or switch not active, we are effectively manual
    if (!autoMode || !sw) {
        // Manual behavior: obey manualRpmTarget
        MetaSense::Settings::setRpmTarget(manualRpmTarget);

        // When fully back in manual (no recording), reset energy and state
        if (!recording) {
            resetEnergy();
            state = State::MANUAL;
        }

        return;
    }

    // From here: autoMode == true and sw == true → AUTO behavior
    if (autoRunStartMs != 0 && (millis() - autoRunStartMs) >= autoRunTimeoutMs) {
        beginSafetyShutdown(rpm);
        state = State::AUTO_TIMEOUT_DOWN;
        return;
    }

    switch (state)
    {
        case State::MANUAL:
        case State::AUTO_IDLE:
        {
            // Enter AUTO sequence
            // If already above start RPM, jump directly to exponential up
            if (rpm >= rpmStart) {
                rpmRampUp.init(rpmStart, rpmEnd, tauUp, dtSeconds);
                startMeasuringIfInWindow(rpm);
                state = State::AUTO_UP_EXP;
            } else {
                state = State::AUTO_UP_LINEAR;
            }
            break;
        }

        case State::AUTO_UP_LINEAR:
        {
            // Linear ramp towards rpmStart
            float commanded = rpm + linearRate * dtSeconds;

            if (commanded >= rpmStart) {
                commanded = rpmStart;

                // Initialize exponential up-sweep
                rpmRampUp.init(rpmStart, rpmEnd, tauUp, dtSeconds);

                // Start recording when entering measurement region
                if (!recording) {
                    startRecording();
                }
                startMeasuringIfInWindow(commanded);

                state = State::AUTO_UP_EXP;
            }

            MetaSense::Settings::setRpmTarget(commanded);
            break;
        }

        case State::AUTO_UP_EXP:
        {
            float commanded = rpmRampUp.update();
            MetaSense::Settings::setRpmTarget(commanded);

            // Update energy accumulator in measurement window
            updateEnergy(rpm, torqueNm);
            break;
        }

        case State::AUTO_DOWN_EXP:
        {
            float commanded = rpmRampDown.update();
            MetaSense::Settings::setRpmTarget(commanded);

            if (commanded <= rpmStart * 1.001f) {
                // Sweep finished normally
                lastRunEnergy = energyMJ;
                runFinished   = true;
                // End auto-run and return to manual control after rampdown.
                autoMode = false;
                panelAuto = false;
                state = State::MANUAL;
            }
            break;
        }

        case State::AUTO_TIMEOUT_DOWN:
        {
            float commanded = rpmRampDown.update();
            MetaSense::Settings::setRpmTarget(commanded);
            setTorqueFeedForward(0.0f);

            if (commanded <= safetyShutdownTargetRpm * 1.001f) {
                completeSafetyShutdown();
            }
            break;
        }

        case State::AUTO_DONE:
        {
            // Legacy state: immediately fall back to manual.
            autoMode = false;
            panelAuto = false;
            state = State::MANUAL;
            break;
        }

        case State::AUTO_ABORT:
        default:
        {
            // Should not normally be here; treat as abort
            abortAutoRun();
            break;
        }
    }
}

} // namespace MetaSense::DynoStateMachine

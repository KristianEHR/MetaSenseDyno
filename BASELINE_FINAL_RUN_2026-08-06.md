# Stable Baseline Template (Final Run)

Date: 2026-08-06

This document records the validated firmware baseline that is considered stable for final-run templating.

## Baseline Identity

- Branch: baseline-1d4-runtime-ui-2026-08-06
- Commit: d4e4795
- Tag: baseline-1d4-runtime-ui-2026-08-06-tag
- Commit title: Baseline: stable 1D4 runtime/UI final-run template

## Scope Included In This Baseline

- 0x1D4 HV status behavior aligned with effective telemetry HV source.
- ChargeStatus initialized/fixed to 0x01 in the 0x1D4 command path.
- Runtime torque target control via UI stepper (+/- 0.1 Nm), defaulting to 0.0 Nm.
- End-to-end command path for runtime torque setpoint updates through WebSocket command handling.
- CAN monitor usability improvements:
  - Collapsible monitor sections.
  - Scrollable monitor container for crowded diagnostics.
- UI and telemetry wiring for 0x1D4 TX/snoop visibility used during validation.

## Validation Context

- Multiple successful build/upload cycles were completed.
- Filesystem upload with the latest UI was completed successfully.
- Final operator confirmation: behavior and UI were reported as working fine.

## How To Reuse This Template

1. Check out the baseline branch:
   git checkout baseline-1d4-runtime-ui-2026-08-06
2. Or check out the exact baseline snapshot by tag:
   git checkout baseline-1d4-runtime-ui-2026-08-06-tag
3. Build/upload using the preferred PlatformIO environment for the target run.

## Optional Publishing (Remote)

If not already pushed:

- git push origin baseline-1d4-runtime-ui-2026-08-06
- git push origin baseline-1d4-runtime-ui-2026-08-06-tag

## Notes

- Treat this baseline as the known-good starting point for final-run iterations.
- Any further experimental changes should branch from this baseline and be re-validated before merge.

# Changelog

## 0.1.0 — 2026-09-24

- Extract the deployed SO-101 → PiPER local teleoperation bridge into an installable Linux package.
- Preserve relative takeover, five-axis mapping with held J4, calibrated wrist-range mapping and absolute gripper aperture control.
- Preserve velocity/acceleration smoothing, fresh-feedback checks, bounded tracking history, explicit preparation and operator ARM.
- Include gs_usb legacy ERROR-ACTIVE notification handling and fault snapshots without ignoring genuine NO_ACK or bus-off.
- Replace private serial/Conda/calibration defaults with portable CLI configuration; support deployed and PyPI LeRobot module layouts.
- Add Chinese documentation, an English quick start, protocol notes, validation scope and offline CI.
- Known limitation: the original intermittent physical CAN NO_ACK cause is still unresolved.

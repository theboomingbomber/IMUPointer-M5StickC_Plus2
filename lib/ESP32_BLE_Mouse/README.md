# IMUPointer BLE Mouse Wrapper

This folder contains a project-local BLE HID mouse wrapper used by the IMUPointer firmware.

## Implementation

- BLE stack: `NimBLE-Arduino`
- API surface: compatible with the `BleMouse` methods used by `src/main.cpp`
- Pairing helper: `startPairingMode()` disconnects peers, clears bonds, and restarts advertising

## License Notes

This wrapper is project code. It depends on the `NimBLE-Arduino` library, which is licensed under Apache-2.0.
See `THIRD_PARTY_NOTICES.md` at the repository root for attribution details.

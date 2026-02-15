# IMUPointer Firmware (M5StickC Plus2)

`IMUPointer` is a PlatformIO firmware project that turns an `M5StickC Plus2` into a BLE air mouse for BLE HID hosts such as Windows and macOS.

## Highlights

- Smooth gyro-based pointer control with light acceleration
- Scroll/click dual-mode on `BtnB` (defaults to scroll)
- Rest lock to stop pointer drift when the device is set down
- Manual recalibration with countdown
- Manual BLE pairing-mode trigger from the on-device menu
- Optimized UI refresh to avoid flicker

## Requirements

- `M5StickC Plus2`
- `PlatformIO` (`pio` CLI)
- USB cable for flashing

## Build

```bash
pio run
```

After build, launcher-ready binaries are exported to `dist/`:

- `dist/m5stickc_plus2-for-m5launcher.bin` (app-only image, use this first in M5Launcher)
- `dist/m5stickc_plus2-full-flash-0x0000.bin` (bootloader + partitions + app merged image for full flashing)

## Flash

```bash
pio run -t upload
```

## M5Launcher

Copy `dist/m5stickc_plus2-for-m5launcher.bin` to your SD card and launch it from `M5Launcher`.
If your launcher workflow needs a full flash image instead, use
`dist/m5stickc_plus2-full-flash-0x0000.bin`.

## Serial Monitor

```bash
pio device monitor -b 115200
```

## First Pairing

1. Power on the device.
2. Open your host Bluetooth settings.
3. Add Bluetooth device and pair with `IMUPointer`.
4. After connect, pointer/click/scroll should be active.

If it pairs but does not control the mouse, remove the old pairing and pair again.

## Controls

### Live Mode

- Tilt/point device: move cursor
- `BtnA`: left click (hold for drag)
- `BtnB` in `SCROLL` mode: hold to scroll
- `BtnB` in `CLICK` mode: right click
- `BtnPWR`: open menu (movement pauses)

### Menu Mode

- `BtnA`: toggle tracking (`ON/OFF`)
- `BtnB` click: toggle `BtnB` mode (`SCROLL/CLICK`)
- `BtnB` hold ~1.2s: force BLE pairing mode (disconnect + advertise)
- `BtnA + BtnB` hold ~1.5s: gyro recalibration (3-second countdown)
- `BtnPWR`: return to live mode

## Tuning

Core motion/UI constants are in `src/main.cpp`, including:

- `kSensitivityX`, `kSensitivityY`
- `kScrollSensitivity`
- `kDeadzoneDps`, `kFilterAlpha`
- `kRestGyroDps`, `kRestEnterMs`, `kRestWakeGyroLateDps`
- `kAccelCurveGain`
- `kRecalibHoldMs`, `kPairingHoldMs`

## Debug Output

The firmware logs state at `115200` baud (about once per second), including:

- BLE connection state
- IMU status and gyro values
- emitted movement/scroll deltas
- button and mode states

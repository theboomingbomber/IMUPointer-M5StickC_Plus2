#include <Arduino.h>
#include <M5Unified.h>
#include <BleMouse.h>

namespace {
constexpr const char* kDeviceName = "IMUPointer";
constexpr const char* kManufacturer = "M5Stack";

constexpr uint32_t kSampleIntervalMs = 8;     // ~125 Hz update loop
constexpr float kSensitivityX = 46.0f;        // Left/right (yaw) multiplier
constexpr float kSensitivityY = 38.0f;        // Up/down (pitch) multiplier
constexpr float kScrollSensitivity = 0.85f;   // Scroll speed when BtnB in scroll mode
constexpr float kDeadzoneDps = 1.20f;         // Ignore tiny gyro drift
constexpr float kFilterAlpha = 0.12f;         // 0..1 low-pass blend factor (lower = smoother)
constexpr float kRestGyroDps = 3.20f;         // Near-still threshold for desk-rest lock
constexpr uint32_t kRestEnterMs = 360;        // How long to be still before rest lock
constexpr float kFlatAccelZMin = 0.90f;       // "Face-up/face-down on desk" accel check
constexpr float kFlatAccelXYMax = 0.30f;
constexpr uint32_t kRestWakeTightenMs = 2200; // After this, wake threshold becomes much stricter
constexpr float kRestWakeGyroEarlyDps = 2.7f;
constexpr float kRestWakeGyroLateDps = 8.8f;
constexpr float kRestPickupTiltG = 0.42f;     // Pick-up detection based on tilt away from flat
constexpr float kRestPickupZMinG = 0.75f;
constexpr float kAccelCurveGain = 0.28f;      // Light speed-up for faster motions
constexpr float kAccelCurveRefDps = 120.0f;
constexpr uint16_t kCalibSamples = 320;       // Startup gyro calibration
constexpr uint32_t kRecalibHoldMs = 1500;     // Hold A+B to recalibrate
constexpr uint32_t kPairingHoldMs = 1200;     // Hold B (in menu) to force pairing mode
constexpr uint32_t kStatusRefreshMs = 240;
constexpr uint32_t kBatteryRefreshMs = 1500;
constexpr uint32_t kDebugRefreshMs = 1000;
constexpr uint32_t kClickStabilizeMs = 140;   // Freeze movement right after left-click press
constexpr float kClickSensitivityScale = 0.30f;
constexpr float kClickDeadzoneDps = 2.80f;
constexpr uint8_t kDisplayRotation = 2;       // 90 degrees clockwise from previous layout

constexpr uint16_t kBgTop = 0x018A;           // Deep teal-blue
constexpr uint16_t kBgBottom = 0x0843;        // Very dark blue-gray
constexpr uint16_t kPanel = 0x10A2;           // Dark slate
constexpr uint16_t kPanel2 = 0x18E3;          // Slightly lighter slate
constexpr uint16_t kTextPrimary = 0xFFFF;     // White
constexpr uint16_t kTextMuted = 0xAD55;       // Gray
constexpr uint16_t kAccent = 0x3E9F;          // Cyan
constexpr uint16_t kGood = 0x07E0;            // Green
constexpr uint16_t kWarn = 0xFD20;            // Amber
constexpr uint16_t kBad = 0xF800;             // Red

BleMouse bleMouse(kDeviceName, kManufacturer, 100);

enum class UiMode : uint8_t {
  AirMouse,
  Menu,
};

enum class BtnBMode : uint8_t {
  RightClick,
  Scroll,
};

struct GyroBias {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

GyroBias g_bias;
float g_filteredX = 0.0f;
float g_filteredY = 0.0f;
float g_accumX = 0.0f;
float g_accumY = 0.0f;
float g_accumWheel = 0.0f;

float g_lastGyroX = 0.0f;
float g_lastGyroY = 0.0f;
float g_lastGyroZ = 0.0f;
int g_lastMoveX = 0;
int g_lastMoveY = 0;
int g_lastWheel = 0;

UiMode g_mode = UiMode::AirMouse;
BtnBMode g_btnBMode = BtnBMode::Scroll;
bool g_trackingEnabled = true;
bool g_recalibLatch = false;
bool g_pairingLatch = false;
bool g_pairingClickSuppress = false;
bool g_restLock = false;
int32_t g_batteryPercent = -1;
float g_batteryPercentFiltered = -1.0f;
bool g_batteryCharging = false;

uint32_t g_lastSampleMs = 0;
uint32_t g_lastStatusMs = 0;
uint32_t g_lastBatteryMs = 0;
uint32_t g_lastDebugMs = 0;
uint32_t g_restCandidateMs = 0;
uint32_t g_restLockSinceMs = 0;
uint32_t g_leftPressStartMs = 0;

bool g_leftDown = false;
bool g_rightDown = false;
bool g_prevConnected = false;
M5Canvas g_canvas(&M5.Display);
bool g_canvasReady = false;

const char* modeToStr(UiMode mode) {
  return mode == UiMode::Menu ? "menu" : "air";
}

const char* btnBModeToStr(BtnBMode mode) {
  return mode == BtnBMode::Scroll ? "scroll" : "right";
}

const char* btnBModeShort(BtnBMode mode) {
  return mode == BtnBMode::Scroll ? "SCROLL" : "CLICK";
}

uint16_t blend565(uint16_t a, uint16_t b, float t) {
  t = constrain(t, 0.0f, 1.0f);
  const uint8_t ar = (a >> 11) & 0x1F;
  const uint8_t ag = (a >> 5) & 0x3F;
  const uint8_t ab = a & 0x1F;
  const uint8_t br = (b >> 11) & 0x1F;
  const uint8_t bg = (b >> 5) & 0x3F;
  const uint8_t bb = b & 0x1F;
  const uint8_t rr = static_cast<uint8_t>(ar + (br - ar) * t);
  const uint8_t rg = static_cast<uint8_t>(ag + (bg - ag) * t);
  const uint8_t rb = static_cast<uint8_t>(ab + (bb - ab) * t);
  return static_cast<uint16_t>((rr << 11) | (rg << 5) | rb);
}

void drawGradientBackground(M5Canvas& canvas, int w, int h) {
  for (int y = 0; y < h; ++y) {
    const float t = static_cast<float>(y) / static_cast<float>(max(1, h - 1));
    canvas.drawFastHLine(0, y, w, blend565(kBgTop, kBgBottom, t));
  }
}

void drawTag(int x, int y, const char* text, uint16_t fill, uint16_t textColor) {
  const int w = M5.Display.textWidth(text) + 10;
  const int h = 14;
  M5.Display.fillRoundRect(x, y, w, h, 4, fill);
  M5.Display.drawRoundRect(x, y, w, h, 4, blend565(fill, TFT_WHITE, 0.25f));
  M5.Display.setTextColor(textColor, fill);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(x + 5, y + 4);
  M5.Display.print(text);
}

void drawChip(M5Canvas& canvas,
              int x, int y, int w, int h,
              const char* key, const char* value,
              bool on, uint16_t onColor) {
  const uint16_t fill = on ? onColor : kPanel2;
  const uint16_t text = on ? TFT_BLACK : kTextPrimary;
  canvas.fillRoundRect(x, y, w, h, 5, fill);
  canvas.drawRoundRect(x, y, w, h, 5, blend565(fill, TFT_WHITE, 0.35f));
  canvas.setTextColor(text, fill);
  canvas.setTextSize(1);
  const bool keyEmpty = (key == nullptr) || (key[0] == '\0');
  if (keyEmpty) {
    const int valueW = canvas.textWidth(value);
    const int valueX = x + max(4, (w - valueW) / 2);
    canvas.setCursor(valueX, y + 6);
    canvas.print(value);
    return;
  }
  canvas.setCursor(x + 5, y + 6);
  canvas.print(key);
  const int valueW = canvas.textWidth(value);
  const int valueX = max(x + 20, x + w - 5 - valueW);
  canvas.setCursor(valueX, y + 6);
  canvas.print(value);
}

void updateBatteryState() {
  const uint32_t now = millis();
  if (now - g_lastBatteryMs < kBatteryRefreshMs) {
    return;
  }
  g_lastBatteryMs = now;

  const int32_t rawLevel = M5.Power.getBatteryLevel();
  if (rawLevel >= 0) {
    const float level = static_cast<float>(constrain(rawLevel, 0, 100));
    if (g_batteryPercentFiltered < 0.0f) {
      g_batteryPercentFiltered = level;
    } else {
      g_batteryPercentFiltered = 0.75f * g_batteryPercentFiltered + 0.25f * level;
    }
    g_batteryPercent = static_cast<int32_t>(lroundf(g_batteryPercentFiltered));
    bleMouse.setBatteryLevel(static_cast<uint8_t>(g_batteryPercent));
  } else {
    g_batteryPercent = -1;
    g_batteryPercentFiltered = -1.0f;
  }

  g_batteryCharging = (static_cast<int>(M5.Power.isCharging()) == 1);
}

void drawBatteryBadge(M5Canvas& canvas, int x, int y, int w, int h) {
  uint16_t color = kTextMuted;
  if (g_batteryPercent >= 0) {
    if (g_batteryCharging) {
      color = kAccent;
    } else if (g_batteryPercent <= 15) {
      color = kBad;
    } else if (g_batteryPercent <= 35) {
      color = kWarn;
    } else {
      color = kGood;
    }
  }

  canvas.fillRoundRect(x, y, w, h, 5, kPanel2);
  canvas.drawRoundRect(x, y, w, h, 5, blend565(color, TFT_WHITE, 0.30f));

  const int iconX = x + 5;
  const int iconY = y + 4;
  const int iconW = 13;
  const int iconH = 8;
  canvas.drawRect(iconX, iconY, iconW, iconH, color);
  canvas.fillRect(iconX + iconW, iconY + 2, 2, 4, color);

  int fillW = 0;
  if (g_batteryPercent > 0) {
    const int32_t pctClamped = (g_batteryPercent > 100) ? 100 : g_batteryPercent;
    fillW = ((iconW - 2) * pctClamped) / 100;
  }
  if (fillW > 0) {
    canvas.fillRect(iconX + 1, iconY + 1, fillW, iconH - 2, color);
  }

  char text[20];
  if (g_batteryPercent < 0) {
    snprintf(text, sizeof(text), "BAT --");
  } else if (g_batteryCharging) {
    snprintf(text, sizeof(text), "CHG %ld%%", static_cast<long>(g_batteryPercent));
  } else {
    snprintf(text, sizeof(text), "BAT %ld%%", static_cast<long>(g_batteryPercent));
  }

  canvas.setTextColor(kTextPrimary, kPanel2);
  canvas.setTextSize(1);
  canvas.setCursor(iconX + iconW + 6, y + 5);
  canvas.print(text);
}

void drawStatusScreen() {
  const bool connected = bleMouse.isConnected();
  const bool imuOk = M5.Imu.isEnabled();
  const int w = M5.Display.width();
  const int h = M5.Display.height();
  const int margin = 6;
  const int headerY = 6;
  const int headerH = 24;
  const int chipGap = 4;
  const int chipW = (w - margin * 2 - chipGap) / 2;
  const int chipH = 18;
  const int row1Y = headerY + headerH + 6;
  const int row2Y = row1Y + chipH + 4;
  const int mainY = row2Y + chipH + 6;
  const int mainH = max(44, h - mainY - margin);

  if (!g_canvasReady || g_canvas.width() != w || g_canvas.height() != h) {
    g_canvas.deleteSprite();
    g_canvas.setColorDepth(16);
    g_canvas.createSprite(w, h);
    g_canvasReady = (g_canvas.width() == w && g_canvas.height() == h);
  }
  if (!g_canvasReady) {
    return;
  }

  auto& cv = g_canvas;
  cv.startWrite();
  cv.setTextWrap(false, false);
  drawGradientBackground(cv, w, h);

  cv.fillRoundRect(margin, headerY, w - margin * 2, headerH, 7, kPanel);
  cv.drawRoundRect(margin, headerY, w - margin * 2, headerH, 7, blend565(kAccent, TFT_WHITE, 0.5f));
  cv.setTextColor(kAccent, kPanel);
  cv.setTextSize(1);
  cv.setCursor(margin + 8, headerY + 8);
  cv.print(kDeviceName);
  const char* topState = connected ? "LINK" : "PAIR";
  cv.setTextColor(connected ? kGood : kWarn, kPanel);
  const int topW = cv.textWidth(topState);
  cv.setCursor(w - margin - 8 - topW, headerY + 8);
  cv.print(topState);

  drawChip(cv, margin, row1Y, chipW, chipH, "", (g_mode == UiMode::Menu) ? "MENU" : "LIVE", g_mode != UiMode::Menu, g_mode == UiMode::Menu ? kWarn : kAccent);
  drawChip(cv, margin + chipW + chipGap, row1Y, chipW, chipH, "", btnBModeShort(g_btnBMode), g_btnBMode == BtnBMode::Scroll, g_btnBMode == BtnBMode::Scroll ? kAccent : kPanel2);
  drawChip(cv, margin, row2Y, chipW, chipH, "TRK", g_trackingEnabled ? "ON" : "OFF", g_trackingEnabled, g_trackingEnabled ? kGood : kWarn);
  drawChip(cv, margin + chipW + chipGap, row2Y, chipW, chipH, "RST", g_restLock ? "LOCK" : "FREE", g_restLock, g_restLock ? kWarn : kGood);

  cv.fillRoundRect(margin, mainY, w - margin * 2, mainH, 8, kPanel);
  cv.drawRoundRect(margin, mainY, w - margin * 2, mainH, 8, blend565(kPanel, TFT_WHITE, 0.35f));
  cv.setTextColor(kTextPrimary, kPanel);
  cv.setTextSize(1);

  int ty = mainY + 9;
  const int tx = margin + 9;
  const int lineStep = 12;

  if (g_mode == UiMode::Menu) {
    cv.setTextColor(kWarn, kPanel);
    cv.setCursor(tx, ty);
    cv.print("MENU PAUSED");
    cv.setTextColor(kTextPrimary, kPanel);
    ty += lineStep + 1;
    cv.drawFastHLine(tx, ty, w - (tx + margin + 4), blend565(kTextMuted, kPanel, 0.5f));
    ty += lineStep - 1;
    cv.setCursor(tx, ty); cv.printf("Track: %s", g_trackingEnabled ? "ON" : "OFF");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.printf("Btn B: %s", btnBModeShort(g_btnBMode));
    ty += lineStep;
    cv.setCursor(tx, ty); cv.printf("IMU: %s", imuOk ? "OK" : "ERR");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("A   toggle track");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("B   toggle B mode");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("Hold B   pair mode");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("A+B recalibrate");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("PWR resume");
  } else if (!connected) {
    cv.setTextColor(kWarn, kPanel);
    cv.setCursor(tx, ty);
    cv.print("PAIR IN WINDOWS BT");
    cv.setTextColor(kTextPrimary, kPanel);
    ty += lineStep + 1;
    cv.drawFastHLine(tx, ty, w - (tx + margin + 4), blend565(kTextMuted, kPanel, 0.5f));
    ty += lineStep - 1;
    cv.setCursor(tx, ty); cv.printf("Btn B: %s", btnBModeShort(g_btnBMode));
    ty += lineStep;
    cv.setCursor(tx, ty); cv.printf("IMU: %s", imuOk ? "OK" : "ERR");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("Add device");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.printf("Name: %s", kDeviceName);
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("PWR menu");
  } else {
    cv.setTextColor(kAccent, kPanel);
    cv.setCursor(tx, ty);
    cv.print("READY");
    cv.setTextColor(kTextPrimary, kPanel);
    ty += lineStep + 1;
    cv.drawFastHLine(tx, ty, w - (tx + margin + 4), blend565(kTextMuted, kPanel, 0.5f));
    ty += lineStep - 1;
    cv.setCursor(tx, ty); cv.printf("Btn B: %s", btnBModeShort(g_btnBMode));
    ty += lineStep;
    cv.setCursor(tx, ty); cv.printf("IMU: %s", imuOk ? "OK" : "ERR");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("A   click / drag");
    ty += lineStep;
    if (g_btnBMode == BtnBMode::Scroll) {
      cv.setCursor(tx, ty); cv.print("B   hold to scroll");
    } else {
      cv.setCursor(tx, ty); cv.print("B   right click");
    }
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("A+B recalibrate");
    ty += lineStep;
    cv.setCursor(tx, ty); cv.print("PWR menu");
  }

  drawBatteryBadge(cv, w - margin - 84, mainY + mainH - 24, 78, 16);

  cv.endWrite();
  cv.pushSprite(0, 0);
}

void drawCalibrationOverlay(const char* headline, const char* detail, uint16_t color) {
  const int w = M5.Display.width();
  const int h = M5.Display.height();
  const int boxW = w - 34;
  const int boxH = 58;
  const int x = (w - boxW) / 2;
  const int y = (h - boxH) / 2;

  M5.Display.startWrite();
  M5.Display.fillRoundRect(x, y, boxW, boxH, 10, kPanel2);
  M5.Display.drawRoundRect(x, y, boxW, boxH, 10, color);
  M5.Display.setTextWrap(false, false);
  M5.Display.setTextColor(color, kPanel2);
  M5.Display.setTextSize((boxW >= 180) ? 2 : 1);
  M5.Display.setCursor(x + 10, y + 10);
  M5.Display.print(headline);
  M5.Display.setTextColor(kTextPrimary, kPanel2);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(x + 10, y + 36);
  M5.Display.print(detail);
  M5.Display.endWrite();
}

bool readGyro(float& x, float& y, float& z) {
  const bool ok = M5.Imu.getGyroData(&x, &y, &z);
  if (ok) {
    g_lastGyroX = x;
    g_lastGyroY = y;
    g_lastGyroZ = z;
  }
  return ok;
}

bool readAccel(float& x, float& y, float& z) {
  return M5.Imu.getAccelData(&x, &y, &z);
}

void resetMotionIntegrators() {
  g_filteredX = 0.0f;
  g_filteredY = 0.0f;
  g_accumX = 0.0f;
  g_accumY = 0.0f;
  g_accumWheel = 0.0f;
}

void releaseAllMouseButtons() {
  if (g_leftDown) {
    bleMouse.release(MOUSE_LEFT);
    g_leftDown = false;
  }
  if (g_rightDown) {
    bleMouse.release(MOUSE_RIGHT);
    g_rightDown = false;
  }
}

void calibrateGyro(bool withCountdown) {
  if (withCountdown) {
    for (int sec = 3; sec > 0; --sec) {
      drawStatusScreen();
      char headline[24];
      snprintf(headline, sizeof(headline), "Recal in %d", sec);
      drawCalibrationOverlay(headline, "Keep still", kWarn);
      Serial.printf("[IMU] recalibration countdown %d\n", sec);
      const uint32_t t0 = millis();
      while (millis() - t0 < 1000) {
        M5.update();
        delay(12);
      }
    }
  }

  drawStatusScreen();
  drawCalibrationOverlay("Calibrating", "Hold still...", kAccent);

  Serial.println("[IMU] calibration started");

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  uint16_t goodSamples = 0;

  for (uint16_t i = 0; i < kCalibSamples; ++i) {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (readGyro(x, y, z)) {
      sumX += x;
      sumY += y;
      sumZ += z;
      ++goodSamples;
    }
    delay(4);
  }

  if (goodSamples > 0) {
    g_bias.x = sumX / static_cast<float>(goodSamples);
    g_bias.y = sumY / static_cast<float>(goodSamples);
    g_bias.z = sumZ / static_cast<float>(goodSamples);
  }
  resetMotionIntegrators();
  g_restLock = false;
  g_restCandidateMs = 0;
  g_restLockSinceMs = 0;

  Serial.printf("[IMU] calibration done samples=%u bias=(%.3f, %.3f, %.3f)\n",
                goodSamples, g_bias.x, g_bias.y, g_bias.z);

  delay(180);
}

void enterPairingMode() {
  releaseAllMouseButtons();
  resetMotionIntegrators();
  const bool ok = bleMouse.startPairingMode();
  Serial.printf("[BLE] pairing mode request -> %s\n", ok ? "started" : "not ready");
  drawStatusScreen();
  drawCalibrationOverlay(ok ? "PAIR MODE" : "PAIR WAIT",
                         ok ? "Scan in Windows BT" : "BLE still starting",
                         ok ? kAccent : kWarn);
}

float applyDeadzone(float value, float deadzone) {
  if (fabsf(value) < deadzone) {
    return 0.0f;
  }
  return value;
}

void handleUiAndModeButtons() {
  if (M5.BtnPWR.wasClicked()) {
    g_mode = (g_mode == UiMode::AirMouse) ? UiMode::Menu : UiMode::AirMouse;
    releaseAllMouseButtons();
    Serial.printf("[UI] mode -> %s\n", modeToStr(g_mode));
  }

  const bool bothHeldForRecalib = M5.BtnA.pressedFor(kRecalibHoldMs) && M5.BtnB.pressedFor(kRecalibHoldMs);
  if (bothHeldForRecalib && !g_recalibLatch) {
    g_recalibLatch = true;
    calibrateGyro(true);
  }
  if (!M5.BtnA.isPressed() || !M5.BtnB.isPressed()) {
    g_recalibLatch = false;
  }

  if (g_mode != UiMode::Menu) {
    return;
  }

  if (M5.BtnA.wasClicked()) {
    g_trackingEnabled = !g_trackingEnabled;
    Serial.printf("[UI] tracking -> %s\n", g_trackingEnabled ? "on" : "paused");
  }

  if (M5.BtnB.wasClicked()) {
    if (g_pairingClickSuppress) {
      g_pairingClickSuppress = false;
    } else {
      g_btnBMode = (g_btnBMode == BtnBMode::RightClick) ? BtnBMode::Scroll : BtnBMode::RightClick;
      Serial.printf("[UI] BtnB mode -> %s\n", btnBModeToStr(g_btnBMode));
    }
  }

  const bool bHeldForPairing = !M5.BtnA.isPressed() && M5.BtnB.pressedFor(kPairingHoldMs);
  if (bHeldForPairing && !g_pairingLatch) {
    g_pairingLatch = true;
    g_pairingClickSuppress = true;
    enterPairingMode();
  }
  if (!M5.BtnB.isPressed()) {
    g_pairingLatch = false;
  }
}

void updateClicks() {
  if (!bleMouse.isConnected() || g_mode == UiMode::Menu) {
    g_leftPressStartMs = 0;
    releaseAllMouseButtons();
    return;
  }

  const bool aPressed = M5.BtnA.isPressed();
  const bool bPressed = M5.BtnB.isPressed();

  if (aPressed != g_leftDown) {
    g_leftDown = aPressed;
    if (g_leftDown) {
      g_leftPressStartMs = millis();
      resetMotionIntegrators();
      bleMouse.press(MOUSE_LEFT);
    } else {
      g_leftPressStartMs = 0;
      bleMouse.release(MOUSE_LEFT);
    }
  }

  if (g_btnBMode == BtnBMode::RightClick) {
    if (bPressed != g_rightDown) {
      g_rightDown = bPressed;
      if (g_rightDown) {
        bleMouse.press(MOUSE_RIGHT);
      } else {
        bleMouse.release(MOUSE_RIGHT);
      }
    }
  } else if (g_rightDown) {
    bleMouse.release(MOUSE_RIGHT);
    g_rightDown = false;
  }
}

void updateMotion() {
  const uint32_t now = millis();
  if (now - g_lastSampleMs < kSampleIntervalMs) {
    return;
  }

  const float dt = (now - g_lastSampleMs) / 1000.0f;
  g_lastSampleMs = now;

  g_lastMoveX = 0;
  g_lastMoveY = 0;
  g_lastWheel = 0;

  if (!g_trackingEnabled || g_mode == UiMode::Menu || !bleMouse.isConnected()) {
    resetMotionIntegrators();
    g_restLock = false;
    g_restCandidateMs = 0;
    g_restLockSinceMs = 0;
    return;
  }

  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  if (!readGyro(gx, gy, gz)) {
    return;
  }

  float activeDeadzone = kDeadzoneDps;
  float sensitivityX = kSensitivityX;
  float sensitivityY = kSensitivityY;

  // Click stabilization: suppress initial shake and reduce movement while holding left-click.
  if (M5.BtnA.isPressed()) {
    if (g_leftPressStartMs && (now - g_leftPressStartMs < kClickStabilizeMs)) {
      resetMotionIntegrators();
      return;
    }
    activeDeadzone = max(activeDeadzone, kClickDeadzoneDps);
    sensitivityX *= kClickSensitivityScale;
    sensitivityY *= kClickSensitivityScale;
  }

  gx = applyDeadzone(gx - g_bias.x, activeDeadzone);
  gy = applyDeadzone(gy - g_bias.y, activeDeadzone);
  gz = applyDeadzone(gz - g_bias.z, activeDeadzone);

  // Desk-rest lock: when device is still and lying flat for a short period,
  // freeze motion so the pointer does not drift while set down.
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  const bool haveAccel = readAccel(ax, ay, az);
  const bool lowGyro = fabsf(gx) < kRestGyroDps && fabsf(gy) < kRestGyroDps && fabsf(gz) < kRestGyroDps;
  const bool flatDesk = haveAccel && fabsf(az) > kFlatAccelZMin && fabsf(ax) < kFlatAccelXYMax && fabsf(ay) < kFlatAccelXYMax;
  if (!g_restLock) {
    if (lowGyro && flatDesk) {
      if (g_restCandidateMs == 0) {
        g_restCandidateMs = now;
      } else if (now - g_restCandidateMs >= kRestEnterMs) {
        g_restLock = true;
        g_restLockSinceMs = now;
        g_restCandidateMs = 0;
        resetMotionIntegrators();
      }
    } else {
      g_restCandidateMs = 0;
    }
  } else {
    const uint32_t lockedFor = now - g_restLockSinceMs;
    const float wakeGyro = (lockedFor >= kRestWakeTightenMs) ? kRestWakeGyroLateDps : kRestWakeGyroEarlyDps;
    const bool pickedUp = haveAccel && (fabsf(ax) > kRestPickupTiltG || fabsf(ay) > kRestPickupTiltG || fabsf(az) < kRestPickupZMinG);
    const bool wakeByGyro = fabsf(gx) > wakeGyro || fabsf(gy) > wakeGyro || fabsf(gz) > wakeGyro;
    if (pickedUp || wakeByGyro) {
      g_restLock = false;
      g_restCandidateMs = 0;
      g_restLockSinceMs = 0;
      resetMotionIntegrators();
    } else {
      resetMotionIntegrators();
      return;
    }
  }

  if (g_btnBMode == BtnBMode::Scroll && M5.BtnB.isPressed()) {
    g_accumWheel += gx * kScrollSensitivity * dt;
    const int wheel = static_cast<int>(lroundf(g_accumWheel));
    if (wheel != 0) {
      g_accumWheel -= static_cast<float>(wheel);
      const int8_t clampedWheel = static_cast<int8_t>(constrain(wheel, -127, 127));
      bleMouse.move(0, 0, clampedWheel, 0);
      g_lastWheel = clampedWheel;
    }
    return;
  }

  // Gyro orientation mapping:
  // X uses yaw-like axis (gz) so left/right feels like pointing.
  // Y keeps pitch-like axis (gx), matching existing up/down feel.
  const float angularSpeed = sqrtf(gz * gz + gx * gx);
  const float accelNorm = constrain(angularSpeed / kAccelCurveRefDps, 0.0f, 1.0f);
  const float accelFactor = 1.0f + kAccelCurveGain * powf(accelNorm, 1.35f);

  const float rawMoveX = -gz * sensitivityX * accelFactor * dt;
  const float rawMoveY = gx * sensitivityY * accelFactor * dt;

  g_filteredX = (1.0f - kFilterAlpha) * g_filteredX + kFilterAlpha * rawMoveX;
  g_filteredY = (1.0f - kFilterAlpha) * g_filteredY + kFilterAlpha * rawMoveY;

  g_accumX += g_filteredX;
  g_accumY += g_filteredY;

  const int moveX = static_cast<int>(lroundf(g_accumX));
  const int moveY = static_cast<int>(lroundf(g_accumY));

  if (moveX != 0 || moveY != 0) {
    g_accumX -= static_cast<float>(moveX);
    g_accumY -= static_cast<float>(moveY);

    const int8_t clampedX = static_cast<int8_t>(constrain(moveX, -127, 127));
    const int8_t clampedY = static_cast<int8_t>(constrain(moveY, -127, 127));
    bleMouse.move(clampedX, clampedY, 0, 0);
    g_lastMoveX = clampedX;
    g_lastMoveY = clampedY;
  }
}

void updateDebugOutput() {
  const uint32_t now = millis();
  if (now - g_lastDebugMs < kDebugRefreshMs) {
    return;
  }
  g_lastDebugMs = now;

  const bool connected = bleMouse.isConnected();
  if (connected != g_prevConnected) {
    Serial.printf("[BLE] %s\n", connected ? "connected" : "disconnected");
    g_prevConnected = connected;
  }

  Serial.printf("[STATE] mode=%s ble=%d imu=%d track=%d rest=%d bmode=%s gyro=(%.2f,%.2f,%.2f) move=(%d,%d,%d) btn(A:%d B:%d P:%d)\n",
                modeToStr(g_mode),
                connected ? 1 : 0,
                M5.Imu.isEnabled() ? 1 : 0,
                g_trackingEnabled ? 1 : 0,
                g_restLock ? 1 : 0,
                btnBModeToStr(g_btnBMode),
                g_lastGyroX, g_lastGyroY, g_lastGyroZ,
                g_lastMoveX, g_lastMoveY, g_lastWheel,
                M5.BtnA.isPressed() ? 1 : 0,
                M5.BtnB.isPressed() ? 1 : 0,
                M5.BtnPWR.isPressed() ? 1 : 0);
}

void updateDisplay() {
  const uint32_t now = millis();
  if (now - g_lastStatusMs < kStatusRefreshMs) {
    return;
  }
  g_lastStatusMs = now;
  drawStatusScreen();
}
}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  cfg.serial_baudrate = 115200;
  cfg.internal_imu = true;
  cfg.output_power = true;
  cfg.fallback_board = m5::board_t::board_M5StickCPlus2;
  M5.begin(cfg);

  Serial.begin(115200);
  delay(40);
  Serial.println("\n[IMUPointer] boot");
  Serial.printf("[BOOT] board=%d imu=%d\n", static_cast<int>(M5.getBoard()), M5.Imu.isEnabled() ? 1 : 0);

  if (!M5.Imu.isEnabled()) {
    M5.In_I2C.begin();
    M5.Imu.begin(&M5.In_I2C, m5::board_t::board_M5StickCPlus2);
    Serial.printf("[BOOT] forced IMU begin -> %d\n", M5.Imu.isEnabled() ? 1 : 0);
  }

  M5.Display.setRotation(kDisplayRotation);
  M5.Display.setTextDatum(top_left);

  calibrateGyro(true);

  bleMouse.begin();
  g_prevConnected = bleMouse.isConnected();
  g_lastSampleMs = millis();
  g_lastStatusMs = 0;
  g_lastBatteryMs = 0;
  g_lastDebugMs = 0;
  updateBatteryState();

  drawStatusScreen();
}

void loop() {
  M5.update();
  handleUiAndModeButtons();
  updateClicks();
  updateMotion();
  updateBatteryState();
  updateDebugOutput();
  updateDisplay();
  delay(1);
}

#include <Preferences.h>

// --- Final TripCore Pinout ---
const int ADC_PIN   = 34;  // ADC1_CH6
const int GAIN_PIN  = 25;  // LOW = 1x, HIGH = 11x
const int SSR_PIN   = 13;  // HIGH = Motor ON, LOW = Motor OFF
const int BTN_PIN   = 4;   // INPUT_PULLUP (Diagonal pins to GND)
const int LED_RED   = 14;
const int LED_GREEN = 26;
const int LED_BLUE  = 27;

Preferences prefs;

bool ssrOn         = false;
bool highGain      = true;   // Live monitor defaults to 11x (matches fan/lamp)
bool flashCal      = false;
bool flashHighGain = false;
float flashBaseMv  = 0.0;

void setRGB(bool r, bool g, bool b) {
  // Common Anode: LOW = ON, HIGH = OFF
  digitalWrite(LED_RED,   r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
}

// 200 samples @ 2 kHz = 100 ms (5 full 50 Hz AC cycles)
float measureAc(float &dcMidOut, float &peakOut) {
  const int N = 200;
  static float mv[N];
  float sum = 0;

  for (int i = 0; i < N; i++) {
    unsigned long t0 = micros();
    mv[i] = analogReadMilliVolts(ADC_PIN);
    sum += mv[i];
    while (micros() - t0 < 500);
  }

  dcMidOut = sum / N;
  float sqSum = 0;
  peakOut = 0;
  for (int i = 0; i < N; i++) {
    float ac = mv[i] - dcMidOut;
    sqSum += ac * ac;
    if (fabs(ac) > peakOut) peakOut = fabs(ac);
  }
  return sqrt(sqSum / N);
}

float avgMeasure(int reps, float &dcAvg, float &pkMax) {
  float rmsSum = 0, dcSum = 0;
  pkMax = 0;
  for (int i = 0; i < reps; i++) {
    float dc, pk;
    rmsSum += measureAc(dc, pk);
    dcSum += dc;
    if (pk > pkMax) pkMax = pk;
  }
  dcAvg = dcSum / reps;
  return rmsSum / reps;
}

void loadFlashInfo() {
  if (prefs.begin("tripcore", true)) {
    flashCal      = prefs.getBool("calibrated", false);
    flashHighGain = prefs.getBool("highGain", false);
    flashBaseMv   = prefs.getFloat("baseRms", 0.0);
    prefs.end();
  } else {
    flashCal = false;
    flashHighGain = false;
    flashBaseMv = 0.0;
  }
  if (flashCal) highGain = flashHighGain;
}

void wipeFlash() {
  prefs.begin("tripcore", false);
  prefs.clear();
  prefs.end();
  loadFlashInfo();
  Serial.println("\n>>> FLASH 'tripcore' WIPED CLEAN (Next main boot will fresh-LEARN) <<<\n");
}

void runRGBTest() {
  Serial.println("\n[TEST] RGB LED: RED -> GREEN -> BLUE -> YELLOW -> CYAN");
  setRGB(true, false, false);  delay(450);
  setRGB(false, true, false);  delay(450);
  setRGB(false, false, true);  delay(450);
  setRGB(true, true, false);   delay(450);
  setRGB(false, true, true);   delay(450);
  setRGB(false, false, false);
}

void runFullAudit() {
  Serial.println("\n========================================================");
  Serial.println("         TRIPCORE FULL HARDWARE & FLASH AUDIT           ");
  Serial.println("========================================================");

  // 1. Button Check
  bool btnLow = (digitalRead(BTN_PIN) == LOW);
  Serial.print("[1] PUSH BUTTON (GPIO 4): ");
  Serial.println(btnLow ? "FAIL -> STUCK LOW! Check diagonal button wiring." : "PASS (UP / HIGH)");

  // 2. Idle / Noise Floor Check (SSR OFF)
  Serial.println("[2] Measuring IDLE Noise Floor (SSR OFF)...");
  digitalWrite(SSR_PIN, LOW);
  digitalWrite(GAIN_PIN, LOW);
  delay(800);
  float dc1Idle, pk1Idle;
  float rms1Idle = avgMeasure(4, dc1Idle, pk1Idle);

  digitalWrite(GAIN_PIN, HIGH);
  delay(700);
  float dc11Idle, pk11Idle;
  float rms11Idle = avgMeasure(4, dc11Idle, pk11Idle);

  Serial.print("    DC_Mid (1x/11x): "); Serial.print(dc1Idle, 0);
  Serial.print(" / "); Serial.print(dc11Idle, 0); Serial.print(" mV -> ");
  if (dc1Idle < 950.0) {
    Serial.println("FAIL -> ZENER DIODE BACKWARDS or SHORTED!");
  } else if (dc1Idle < 1450.0 || dc1Idle > 1880.0 || fabs(dc1Idle - dc11Idle) > 150.0) {
    Serial.println("FAIL -> Check LM358 +5V/GND, 10k/10k divider, or 10uF cap!");
  } else {
    Serial.println("PASS");
  }

  Serial.print("    Idle Noise (1x / 11x): ");
  Serial.print(rms1Idle, 1); Serial.print(" mV / ");
  Serial.print(rms11Idle, 1); Serial.print(" mV -> ");
  if (rms11Idle > 65.0) {
    Serial.println("FAIL -> HUGE NOISE FLOOR! Floating CT jack, loose GND, or AC wire touching breadboard.");
  } else {
    Serial.println("PASS");
  }

  // 3. Loaded Signal + Auto-Gain Check (SSR ON)
  Serial.println("[3] Turning SSR ON (2.0s spin-up) — confirm load physically turns ON!");
  digitalWrite(SSR_PIN, HIGH);
  digitalWrite(GAIN_PIN, LOW);
  delay(2000);
  float dc1Load, pk1Load;
  float rms1Load = avgMeasure(5, dc1Load, pk1Load);

  digitalWrite(GAIN_PIN, HIGH);
  delay(700);
  float dc11Load, pk11Load;
  float rms11Load = avgMeasure(5, dc11Load, pk11Load);

  Serial.print("    Loaded RMS (1x / 11x): ");
  Serial.print(rms1Load, 1); Serial.print(" mV / ");
  Serial.print(rms11Load, 1); Serial.println(" mV");

  float signalDelta11 = rms11Load - rms11Idle;
  Serial.print("    CT Sensor Signal Check: ");
  if (signalDelta11 < 25.0 && rms1Load < 25.0) {
    Serial.println("FAIL -> ZERO REAL SIGNAL DETECTED!");
    Serial.println("      Causes: (a) CT jack loose, (b) BOTH Live+Neutral inside CT,");
    Serial.println("      (c) Load is a tiny LED bulb (<15W), or (d) SSR didn't power load.");
  } else if (rms11Load < rms11Idle * 2.2 && rms1Load < 65.0) {
    Serial.println("WARN -> WEAK SIGNAL-TO-NOISE! Add 1 more wire pass or use higher-watt load.");
  } else {
    Serial.println("PASS (Clean AC current detected)");
  }

  // 4. Auto-Gain Ratio Check
  float ratio = (rms1Load > 1.0) ? (rms11Load / rms1Load) : 0.0;
  Serial.print("[4] AUTO-GAIN RATIO (11x / 1x): ");
  Serial.print(ratio, 2); Serial.print("x -> ");
  if (signalDelta11 < 25.0) {
    Serial.println("SKIPPED (Fix CT signal above first)");
  } else if (rms11Load > 950.0) {
    Serial.println("INFO -> 11x CLIPPING (Heavy load — 1x gain is correct for this load)");
  } else if (ratio >= 6.5 && ratio <= 12.5) {
    Serial.println("PASS (LM358 + BC547 healthy)");
  } else {
    Serial.println("FAIL -> Check BC547 E/B/C pins, 4.7k base resistor, 10k/100k, or 10uF cap!");
  }

  // 5. Flash Calibration Sanity Check
  loadFlashInfo();
  Serial.print("[5] SAVED FLASH ('tripcore'): ");
  if (!flashCal) {
    Serial.println("EMPTY (Not calibrated yet)");
  } else {
    float liveAtSavedGain = flashHighGain ? rms11Load : rms1Load;
    float pct = (flashBaseMv > 1.0) ? (liveAtSavedGain / flashBaseMv) * 100.0 : 0.0;
    Serial.print("Gain="); Serial.print(flashHighGain ? "11x" : "1x");
    Serial.print(" | Saved I_base="); Serial.print(flashBaseMv, 1);
    Serial.print("mV | Current Load="); Serial.print(pct, 0); Serial.print("% -> ");

    if (flashBaseMv < rms11Idle * 1.5 && flashHighGain) {
      Serial.println("FAIL -> GARBAGE I_base SAVED ON NOISE! Send 'w' to wipe Flash!");
    } else if (pct < 80.0 || pct > 120.0) {
      Serial.println("WARN -> STALE BASELINE! Current load != 100% of Flash. Re-run LEARN!");
    } else {
      Serial.println("PASS (Flash matches current load)");
    }
  }

  // Leave SSR ON at saved/11x gain for live wire-loop testing
  ssrOn = true;
  highGain = flashCal ? flashHighGain : true;
  digitalWrite(GAIN_PIN, highGain ? HIGH : LOW);
  Serial.println("========================================================");
  Serial.println("LIVE MONITOR ACTIVE: Move CT wires now and watch RMS & Zone.\n");
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(GAIN_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(SSR_PIN, LOW);
  loadFlashInfo();
  digitalWrite(GAIN_PIN, highGain ? HIGH : LOW);

  delay(400);
  Serial.println("\n=== TRIPCORE DOCTOR READY ===");
  Serial.println("Commands: [a] RUN FULL 4-SEC AUTO-AUDIT   [s] Toggle SSR ON/OFF");
  Serial.println("          [g] Toggle 1x/11x Gain          [w] Wipe Saved Flash");
  Serial.println("          [c] Test RGB LED\n");

  runRGBTest();

  Serial.print("Current Flash -> ");
  if (flashCal) {
    Serial.print("Calibrated | Gain: "); Serial.print(flashHighGain ? "11x" : "1x");
    Serial.print(" | I_base: "); Serial.print(flashBaseMv, 1); Serial.println(" mV");
  } else {
    Serial.println("NOT CALIBRATED");
  }
  Serial.println(">>> Make sure your normal load is wired in the CT, then send 'a' <<<\n");
}

void loop() {
  if (Serial.available()) {
    char cmd = tolower(Serial.read());
    if (cmd == 'a') {
      runFullAudit();
    } else if (cmd == 's') {
      ssrOn = !ssrOn;
      digitalWrite(SSR_PIN, ssrOn ? HIGH : LOW);
      Serial.print(">>> SSR: "); Serial.println(ssrOn ? "ON" : "OFF");
    } else if (cmd == 'g') {
      highGain = !highGain;
      digitalWrite(GAIN_PIN, highGain ? HIGH : LOW);
      delay(400);
      Serial.print(">>> GAIN: "); Serial.println(highGain ? "11x" : "1x");
    } else if (cmd == 'w') {
      wipeFlash();
    } else if (cmd == 'c') {
      runRGBTest();
    }
  }

  float dcMid, peakMv;
  float rmsMv = measureAc(dcMid, peakMv);
  bool btnDown = (digitalRead(BTN_PIN) == LOW);

  // Determine exact fault zone relative to saved Flash I_base
  const char* zone = "NO_FLASH";
  float pct = 0.0;
  if (flashCal && flashBaseMv > 1.0) {
    if (highGain != flashHighGain) {
      zone = "GAIN_MISMATCH";
    } else {
      pct = (rmsMv / flashBaseMv) * 100.0;
      if (pct >= 175.0)      zone = "JAM (RED)";
      else if (pct >= 125.0) zone = "OVERLOAD (YEL)";
      else if (pct < 72.0)   zone = "DRY_RUN (BLU)";
      else                   zone = "NORMAL (GRN)";
    }
  }

  // Mirror live zone on RGB LED
  if (btnDown)                    setRGB(false, true, true);   // Cyan = Button pressed
  else if (!ssrOn)                setRGB(false, false, false); // Off = SSR off
  else if (pct >= 175.0)          setRGB(true, false, false);  // Red
  else if (pct >= 125.0)          setRGB(true, true, false);   // Yellow
  else if (flashCal && pct < 72.0) setRGB(false, false, true); // Blue
  else                            setRGB(false, true, false);  // Green

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 450) {
    lastPrint = millis();
    Serial.print("SSR:"); Serial.print(ssrOn ? "ON " : "OFF");
    Serial.print(" | "); Serial.print(highGain ? "11x" : " 1x");
    Serial.print(" | DC:"); Serial.print(dcMid, 0); Serial.print("mV");
    Serial.print(" | RMS:"); Serial.print(rmsMv, 1); Serial.print("mV");
    if (flashCal && highGain == flashHighGain) {
      Serial.print(" / "); Serial.print(flashBaseMv, 1);
      Serial.print("mV ("); Serial.print(pct, 0); Serial.print("%)");
    }
    Serial.print(" | Zone:"); Serial.print(zone);
    Serial.print(" | BTN:"); Serial.println(btnDown ? "DOWN" : "UP");
  }
}
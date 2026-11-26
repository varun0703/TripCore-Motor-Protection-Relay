#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "TripCore"
#define BLYNK_AUTH_TOKEN    "YOUR_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Preferences.h>

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

// --- Pin Definitions (Final TripCore Hardware — No Buzzer) ---
const int ADC_PIN   = 34;  // ADC1_CH6 (Safe with Wi-Fi active)
const int GAIN_PIN  = 25;  // LOW = 1x gain, HIGH = 11x gain
const int SSR_PIN   = 13;  // HIGH = Motor ON, LOW = Motor OFF
const int BTN_PIN   = 4;   // Push button to GND (Diagonal pins, INPUT_PULLUP)
const int LED_RED   = 14;
const int LED_GREEN = 26;
const int LED_BLUE  = 27;

// --- 3-Tier Protection Tuning ---
const unsigned long INRUSH_BYPASS_MS = 6000;  // 6s startup surge mask
const float AUTO_GAIN_1X_LIMIT_MV    = 65.0;  // <65mV at 1x -> switch to 11x
const float DRYRUN_RATIO             = 0.72;  // <72% = Dry Run (Blue)
const float OVERLOAD_RATIO           = 1.25;  // 125% - 175% = I^2t Overload (Yellow)
const float JAM_RATIO                = 1.75;  // >=175% = Locked-Rotor Jam (Red)
const float THERMAL_TRIP_LIMIT       = 3.5;   // I^2*t accumulator trip point
const unsigned long DRYRUN_TIME_MS   = 2500;  // 2.5s sustained under-load
const unsigned long JAM_TIME_MS      = 250;   // 0.25s sustained stall

Preferences prefs;

enum SystemState {
  STATE_WAIT_AC,
  STATE_RUNNING,
  STATE_TRIP_DRYRUN,
  STATE_TRIP_OVERLOAD,
  STATE_TRIP_JAM,
  STATE_REMOTE_OFF
};
SystemState sysState = STATE_WAIT_AC;

bool highGain = false;
bool isCalibrated = false;
float baseRmsMv = 0.0;
float thermalBucket = 0.0;
unsigned long motorStartMs = 0;
unsigned long dryRunStartMs = 0;
unsigned long jamStartMs = 0;
unsigned long lastSampleMs = 0;
float simMultiplier = 1.0;

// Common Anode RGB LED (Longest leg to 3.3V -> LOW = ON, HIGH = OFF)
void setRGB(bool r, bool g, bool b) {
  digitalWrite(LED_RED,   r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b ? LOW : HIGH);
}

// Sync Blynk Switch (V0), Status Text (V4), and Color LED (V5)
void pushBlynkState(const char* statusText, const char* ledHexColor, int ssrSwitchVal) {
  if (!Blynk.connected()) return;
  Blynk.virtualWrite(V0, ssrSwitchVal);
  Blynk.virtualWrite(V4, statusText);
  Blynk.setProperty(V5, "color", ledHexColor);
  Blynk.virtualWrite(V5, 255);
}

// Sample 200 points at 2 kHz (100 ms = 5 full 50 Hz AC cycles)
float measureAcRmsMv(float &dcMidOut, float &peakOut) {
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

void armMotor() {
  simMultiplier = 1.0;
  thermalBucket = 0.0;
  dryRunStartMs = 0;
  jamStartMs = 0;
  sysState = STATE_WAIT_AC;
  digitalWrite(SSR_PIN, HIGH);
  setRGB(false, true, false);  // Green
  Serial.println(">>> SSR ARMED (Motor ON / Waiting for AC current) <<<");
  pushBlynkState("ARMED / WAIT AC", "#2ECC71", 1);
}

void remoteShutdownMotor() {
  digitalWrite(SSR_PIN, LOW);  // Cut AC power immediately
  sysState = STATE_REMOTE_OFF;
  simMultiplier = 1.0;
  thermalBucket = 0.0;
  dryRunStartMs = 0;
  jamStartMs = 0;
  setRGB(true, false, true);   // Purple = Intentionally OFF via Remote
  Serial.println(">>> MOTOR STOPPED REMOTELY VIA BLYNK (V0 = 0) <<<");
  pushBlynkState("REMOTE OFF", "#9B59B6", 0);
  if (Blynk.connected()) {
    Blynk.virtualWrite(V1, 0.0);
    Blynk.virtualWrite(V2, 0.0);
    Blynk.virtualWrite(V3, 0.0);
  }
}

void tripMotor(SystemState faultType, const char* reason, float tripRatio) {
  // 1. SAFETY FIRST: Cut 230V AC before any network call
  digitalWrite(SSR_PIN, LOW);
  sysState = faultType;
  simMultiplier = 1.0;

  // 2. Local RGB LED + Serial
  const char* blynkLabel = "TRIPPED";
  const char* blynkColor = "#FF0000";

  if (faultType == STATE_TRIP_JAM) {
    setRGB(true, false, false);  // Red
    blynkLabel = "TRIP: LOCKED-ROTOR JAM";
    blynkColor = "#FF2E2E";
  } else if (faultType == STATE_TRIP_OVERLOAD) {
    setRGB(true, true, false);   // Yellow
    blynkLabel = "TRIP: I2t OVERLOAD";
    blynkColor = "#F1C40F";
  } else {
    setRGB(false, false, true);  // Blue
    blynkLabel = "TRIP: DRY RUN";
    blynkColor = "#3498DB";
  }

  Serial.print("!!! TRIPPED: ");
  Serial.print(reason);
  Serial.println(" | Short-Press Button, 'r', or Toggle V0 ON to Reset !!!");

  // 3. Push Fault Spike + Status + Phone Notification to Blynk
  if (Blynk.connected()) {
    Blynk.virtualWrite(V1, tripRatio * 100.0);  // Show exact fault level on SuperChart
    Blynk.virtualWrite(V3, thermalBucket);
    pushBlynkState(blynkLabel, blynkColor, 0);
    Blynk.logEvent("motor_fault", reason);
  }
}

void runLearnCalibration() {
  if (sysState != STATE_RUNNING && sysState != STATE_WAIT_AC) {
    armMotor();
    delay(3000);
  }

  Serial.println("\n=== LEARN CALIBRATION STARTED ===");
  setRGB(false, true, true);  // Cyan
  pushBlynkState("LEARNING BASELINE...", "#00E5FF", 1);

  highGain = false;
  digitalWrite(GAIN_PIN, LOW);
  delay(700);

  float dc, pk;
  float rms1x = 0;
  for (int i = 0; i < 5; i++) rms1x += measureAcRmsMv(dc, pk);
  rms1x /= 5.0;

  Serial.print("1x Baseline RMS: ");
  Serial.print(rms1x, 2);
  Serial.println(" mV");

  if (rms1x < AUTO_GAIN_1X_LIMIT_MV) {
    highGain = true;
    digitalWrite(GAIN_PIN, HIGH);
    delay(700);
    Serial.println("Auto-Gain Selected: 11x (High Resolution)");
  } else {
    highGain = false;
    digitalWrite(GAIN_PIN, LOW);
    Serial.println("Auto-Gain Selected: 1x (High Current Range)");
  }

  float learned = 0;
  for (int i = 0; i < 8; i++) learned += measureAcRmsMv(dc, pk);
  baseRmsMv = learned / 8.0;
  isCalibrated = true;

  prefs.begin("tripcore", false);
  prefs.putBool("calibrated", true);
  prefs.putBool("highGain", highGain);
  prefs.putFloat("baseRms", baseRmsMv);
  prefs.end();

  thermalBucket = 0.0;
  dryRunStartMs = 0;
  jamStartMs = 0;
  sysState = STATE_RUNNING;
  motorStartMs = millis() - INRUSH_BYPASS_MS;
  setRGB(false, true, false);  // Solid Green

  Serial.print("=== LEARN COMPLETE | Gain: ");
  Serial.print(highGain ? "11x" : "1x");
  Serial.print(" | Saved I_base: ");
  Serial.print(baseRmsMv, 2);
  Serial.println(" mV ===\n");

  pushBlynkState(highGain ? "PROTECT ACTIVE (11x)" : "PROTECT ACTIVE (1x)", "#2ECC71", 1);
}

// --- Blynk Remote Motor Switch (V0: 0 = OFF, 1 = ON / Reset Trip) ---
BLYNK_WRITE(V0) {
  int cmd = param.asInt();
  if (cmd == 0) {
    remoteShutdownMotor();
  } else if (cmd == 1) {
    if (sysState != STATE_RUNNING && sysState != STATE_WAIT_AC) {
      armMotor();
    }
  }
}

// Sync hardware truth to app whenever Blynk connects (never overrides a tripped motor)
BLYNK_CONNECTED() {
  Serial.println(">>> BLYNK CLOUD CONNECTED <<<");
  if (sysState == STATE_RUNNING) {
    pushBlynkState(highGain ? "PROTECT ACTIVE (11x)" : "PROTECT ACTIVE (1x)", "#2ECC71", 1);
  } else if (sysState == STATE_WAIT_AC) {
    pushBlynkState("ARMED / WAIT AC", "#2ECC71", 1);
  } else if (sysState == STATE_REMOTE_OFF) {
    pushBlynkState("REMOTE OFF", "#9B59B6", 0);
  } else if (sysState == STATE_TRIP_JAM) {
    pushBlynkState("TRIP: LOCKED-ROTOR JAM", "#FF2E2E", 0);
  } else if (sysState == STATE_TRIP_OVERLOAD) {
    pushBlynkState("TRIP: I2t OVERLOAD", "#F1C40F", 0);
  } else {
    pushBlynkState("TRIP: DRY RUN", "#3498DB", 0);
  }
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

  prefs.begin("tripcore", true);
  isCalibrated = prefs.getBool("calibrated", false);
  highGain     = prefs.getBool("highGain", false);
  baseRmsMv    = prefs.getFloat("baseRms", 0.0);
  prefs.end();

  digitalWrite(GAIN_PIN, highGain ? HIGH : LOW);

  setRGB(true, true, true);
  delay(400);

  // Non-blocking Wi-Fi + Blynk init (Local protection runs immediately even if Wi-Fi is down)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);

  Serial.println("\n--- TRIPCORE (3-TIER EDGE + BLYNK IoT) READY ---");
  if (isCalibrated) {
    Serial.print("Loaded Flash Calibration -> Gain: ");
    Serial.print(highGain ? "11x" : "1x");
    Serial.print(" | I_base: ");
    Serial.print(baseRmsMv, 1);
    Serial.println(" mV");
  }
  Serial.println("Serial Keys: [l] LEARN  [d] Dry-Run  [o] Overload  [j] Jam  [r] Reset\n");

  armMotor();
}

void loop() {
  unsigned long now = millis();

  // --- 0. Non-Blocking Blynk Engine ---
  if (Blynk.connected()) {
    Blynk.run();
  } else {
    // Only attempt brief reconnect every 15s when NO fault is actively developing
    static unsigned long lastNetTry = 0;
    bool faultDeveloping = (jamStartMs != 0 || dryRunStartMs != 0 || thermalBucket > 0.1);
    if (!faultDeveloping && (now - lastNetTry >= 15000)) {
      lastNetTry = now;
      if (WiFi.status() == WL_CONNECTED) {
        Blynk.connect(500);  // Max 0.5s attempt — never stalls protection
      } else {
        WiFi.begin(ssid, pass);
      }
    }
  }

  // --- 1. Non-Blocking Push Button Handler ---
  static bool btnInit = false;
  static bool btnWasDown = false;
  static bool learnHandled = false;
  static unsigned long btnDownMs = 0;

  bool btnDown = (digitalRead(BTN_PIN) == LOW);

  if (!btnInit) {
    btnInit = true;
    if (btnDown) {
      btnWasDown = true;
      learnHandled = true;
      Serial.println("WARNING: GPIO 4 stuck LOW at boot! Check diagonal button wiring.");
    }
  }

  if (btnDown && !btnWasDown) {
    btnDownMs = now;
    learnHandled = false;
  } else if (btnDown && btnWasDown) {
    if (!learnHandled && (now - btnDownMs >= 2500)) {
      learnHandled = true;
      runLearnCalibration();
      return;
    }
  } else if (!btnDown && btnWasDown) {
    if (!learnHandled && (now - btnDownMs > 40)) {
      if (sysState != STATE_RUNNING && sysState != STATE_WAIT_AC) {
        armMotor();
      }
    }
  }
  btnWasDown = btnDown;

  // --- 2. Serial Backup Command Handler ---
  if (Serial.available()) {
    char cmd = tolower(Serial.read());
    if (cmd == 'l') runLearnCalibration();
    else if (cmd == 'r') armMotor();
    else if (cmd == 'd' && sysState == STATE_RUNNING) {
      Serial.println(">>> SIMULATING DRY RUN (50% Load) <<<");
      simMultiplier = 0.50;
    } else if (cmd == 'o' && sysState == STATE_RUNNING) {
      Serial.println(">>> SIMULATING THERMAL OVERLOAD (145% Load) <<<");
      simMultiplier = 1.45;
    } else if (cmd == 'j' && sysState == STATE_RUNNING) {
      Serial.println(">>> SIMULATING LOCKED-ROTOR JAM (200% Load) <<<");
      simMultiplier = 2.00;
    }
  }

  // --- 3. Measure & Evaluate 3-Tier Protection Logic ---
  float dcMid, peakMv;
  float rawRmsMv = measureAcRmsMv(dcMid, peakMv);
  float effRmsMv = rawRmsMv * simMultiplier;
  float dtSec = (lastSampleMs == 0) ? 0.1 : (now - lastSampleMs) / 1000.0;
  if (dtSec > 0.5) dtSec = 0.1;  // Prevent dt spike during brief Wi-Fi reconnect
  lastSampleMs = now;

  float wakeThresh = highGain ? 35.0 : 16.5;
  if (sysState == STATE_WAIT_AC && rawRmsMv > wakeThresh) {
    sysState = STATE_RUNNING;
    motorStartMs = now;
    Serial.println(">>> MOTOR CURRENT DETECTED (6.0s Inrush Bypass Started) <<<");
    pushBlynkState("STARTUP INRUSH MASK", "#2ECC71", 1);
  }

  if (sysState == STATE_RUNNING && highGain && rawRmsMv > 780.0) {
    highGain = false;
    digitalWrite(GAIN_PIN, LOW);
    baseRmsMv /= 11.0;
    delay(650);
    Serial.println(">>> AUTO-DOWNSHIFTED TO 1x GAIN <<<");
    return;
  }

  bool inStartup = (sysState == STATE_RUNNING) && ((now - motorStartMs) < INRUSH_BYPASS_MS);
  static bool wasInStartup = false;
  if (wasInStartup && !inStartup && sysState == STATE_RUNNING) {
    pushBlynkState(highGain ? "PROTECT ACTIVE (11x)" : "PROTECT ACTIVE (1x)", "#2ECC71", 1);
  }
  wasInStartup = inStartup;

  if (sysState == STATE_RUNNING && !inStartup && !isCalibrated) {
    runLearnCalibration();
    return;
  }

  float ratio = (baseRmsMv > 1.0) ? (effRmsMv / baseRmsMv) : 1.0;
  float crestFactor = (rawRmsMv > 15.0) ? (peakMv / rawRmsMv) : 0.0;

  if (sysState == STATE_RUNNING && !inStartup && isCalibrated) {
    // Tier 1: Hard Locked-Rotor / Jam (>= 175% for 0.25s -> Red)
    if (ratio >= JAM_RATIO) {
      if (jamStartMs == 0) jamStartMs = now;
      if (now - jamStartMs >= JAM_TIME_MS) {
        tripMotor(STATE_TRIP_JAM, "LOCKED-ROTOR JAM (>=175% Instant Trip)", ratio);
        return;
      }
    } else {
      jamStartMs = 0;
    }

    // Tier 2: Sustained Thermal Overload (125% - 175% I^2*t -> Yellow)
    if (ratio >= OVERLOAD_RATIO) {
      float excess = (ratio * ratio) - 1.0;
      thermalBucket += excess * dtSec;
      if (thermalBucket >= THERMAL_TRIP_LIMIT) {
        tripMotor(STATE_TRIP_OVERLOAD, "SUSTAINED OVERLOAD (I^2t Thermal Limit)", ratio);
        return;
      }
    } else {
      thermalBucket = max(0.0f, thermalBucket - 0.5f * dtSec);
    }

    // Tier 3: Dry Run / Under-Load (< 72% for 2.5s -> Blue)
    if (ratio < DRYRUN_RATIO) {
      if (dryRunStartMs == 0) dryRunStartMs = now;
      if (now - dryRunStartMs >= DRYRUN_TIME_MS) {
        tripMotor(STATE_TRIP_DRYRUN, "DRY RUN / UNDER-LOAD DROP (<72%)", ratio);
        return;
      }
    } else {
      dryRunStartMs = 0;
    }
  }

  // --- 4. Push Live Telemetry to Blynk Once Every 1.0 Second ---
  static unsigned long lastBlynkPush = 0;
  if (Blynk.connected() && (now - lastBlynkPush >= 1000)) {
    lastBlynkPush = now;
    if (sysState == STATE_RUNNING) {
      Blynk.virtualWrite(V1, ratio * 100.0);
      Blynk.virtualWrite(V2, effRmsMv);
      Blynk.virtualWrite(V3, thermalBucket);
      Blynk.virtualWrite(V6, crestFactor);
    } else {
      // Clean 0% drop on SuperChart when SSR is OFF
      Blynk.virtualWrite(V1, 0.0);
      Blynk.virtualWrite(V2, 0.0);
      Blynk.virtualWrite(V3, thermalBucket);
      Blynk.virtualWrite(V6, 0.0);
    }
  }

  // --- 5. Local Serial Telemetry ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 400) {
    lastPrint = now;
    if (sysState == STATE_WAIT_AC) Serial.print("[WAIT_AC] ");
    else if (sysState == STATE_REMOTE_OFF) Serial.print("[REM_OFF] ");
    else if (sysState == STATE_RUNNING) Serial.print(inStartup ? "[STARTUP] " : "[PROTECT] ");
    else Serial.print("[TRIPPED] ");

    Serial.print(Blynk.connected() ? "NET:OK | " : "NET:-- | ");
    Serial.print("Gain:"); Serial.print(highGain ? "11x" : "1x");
    Serial.print(" | RMS:"); Serial.print(effRmsMv, 1);
    Serial.print("/"); Serial.print(baseRmsMv, 1); Serial.print("mV (");
    Serial.print(ratio * 100.0, 0); Serial.print("%)");
    Serial.print(" | Heat:"); Serial.print(thermalBucket, 2);
    Serial.print(" | CF:"); Serial.println(crestFactor, 2);
  }
}
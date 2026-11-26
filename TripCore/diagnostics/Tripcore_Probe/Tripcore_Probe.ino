// --- Plug ONE loose jumper wire into GPIO 35 to use as a live Probe ---
const int PROBE_PIN = 35;  // Handheld probe wire (ADC1_CH7)
const int OUT_PIN   = 34;  // Final circuit output (ADC1_CH6)
const int GAIN_PIN  = 25;  // HIGH = 11x, LOW = 1x
const int SSR_PIN   = 13;  // HIGH = Fan ON, LOW = Fan OFF

bool ssrOn    = true;   // Starts ON so fan runs immediately
bool highGain = true;   // Starts at 11x so we can test the gain stage live

// Measure DC midpoint + True AC_RMS over 5 full 50 Hz cycles (100 ms)
float measurePin(int pin, float &dcOut) {
  const int N = 200;
  static float mv[N];
  float sum = 0;

  for (int i = 0; i < N; i++) {
    unsigned long t0 = micros();
    mv[i] = analogReadMilliVolts(pin);
    sum += mv[i];
    while (micros() - t0 < 500);
  }

  dcOut = sum / N;
  float sqSum = 0;
  for (int i = 0; i < N; i++) {
    float ac = mv[i] - dcOut;
    sqSum += ac * ac;
  }
  return sqrt(sqSum / N);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(GAIN_PIN, OUTPUT);
  pinMode(SSR_PIN, OUTPUT);

  digitalWrite(SSR_PIN, HIGH);   // Fan ON
  digitalWrite(GAIN_PIN, HIGH);  // 11x Gain ON

  delay(400);
  Serial.println("\n======================================================");
  Serial.println("   TRIPCORE 1-WIRE SIGNAL PROBE (Jumper in GPIO 35)   ");
  Serial.println("======================================================");
  Serial.println("WARNING: NEVER touch 230V AC, SSR AC pins, or LM358 Pin 8 (+5V)!");
  Serial.println("Keys: [g] Toggle 1x/11x   [s] Toggle Fan ON/OFF\n");
}

void loop() {
  if (Serial.available()) {
    char c = tolower(Serial.read());
    if (c == 'g') {
      highGain = !highGain;
      digitalWrite(GAIN_PIN, highGain ? HIGH : LOW);
      Serial.print(">>> GAIN = "); Serial.println(highGain ? "11x (GPIO 25 HIGH)" : "1x (GPIO 25 LOW)");
    } else if (c == 's') {
      ssrOn = !ssrOn;
      digitalWrite(SSR_PIN, ssrOn ? HIGH : LOW);
      Serial.print(">>> FAN SSR = "); Serial.println(ssrOn ? "ON" : "OFF");
    }
  }

  float pDc, oDc;
  float pAc = measurePin(PROBE_PIN, pDc);
  float oAc = measurePin(OUT_PIN, oDc);

  Serial.print(ssrOn ? "FAN:ON  | " : "FAN:OFF | ");
  Serial.print(highGain ? "11x | " : " 1x | ");
  Serial.print("PROBE(G35) -> DC:"); Serial.print(pDc, 0);
  Serial.print("mV  AC:"); Serial.print(pAc, 1); Serial.print("mV");
  Serial.print("   ||   OUT(G34) -> DC:"); Serial.print(oDc, 0);
  Serial.print("mV  AC:"); Serial.print(oAc, 1); Serial.println("mV");
  delay(200);
}
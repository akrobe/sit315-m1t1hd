// QP2_ExternalInterrupt.ino — D2 attachInterrupt toggles "armed"; D8 polled; LED = armed && B
// Serial: 115200

const byte PIN_BTN_A = 2;   // D2 -> INT0 (attachInterrupt)
const byte PIN_BTN_B = 8;   // D8 -> polled
const byte PIN_LED   = 12;  // set 13 if using the built-in LED

volatile bool flagArmToggle = false;
bool armed = false;
bool lastBHigh = true;

unsigned long lastArmToggleMs = 0;
const unsigned long ARM_LOCKOUT_MS = 80; // debounce/lockout in main loop

void isrArmFalling() {
  flagArmToggle = true;   // ISR does the minimum
}

void setup() {
  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  Serial.println("[QP2] INT0 + two-condition logic");
  Serial.println("A on D2 toggles 'armed' (interrupt). B on D8 is polled. LED = armed && B.");

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_A), isrArmFalling, FALLING);
}

void loop() {
  // Handle any pending INT0 toggle (debounced by lockout)
  if (flagArmToggle) {
    noInterrupts(); flagArmToggle = false; interrupts();
    unsigned long now = millis();
    if (now - lastArmToggleMs >= ARM_LOCKOUT_MS) {
      lastArmToggleMs = now;
      armed = !armed;
      Serial.print("INT0: ARMED -> ");
      Serial.println(armed ? "true" : "false");
    }
  }

  // Read Button B (LOW = pressed)
  bool bPressed = (digitalRead(PIN_BTN_B) == LOW);

  // Edge log for B
  bool bHigh = !bPressed;
  if (bHigh != lastBHigh) {
    lastBHigh = bHigh;
    Serial.println(bHigh ? "BTN_B RELEASED" : "BTN_B PRESSED");
  }

  // Drive LED
  bool ledOn = (armed && bPressed);
  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);

  // Status once a second
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 1000UL) {
    last = now;
    Serial.print("Status: armed="); Serial.print(armed ? "true" : "false");
    Serial.print(", BTN_B="); Serial.print(bPressed ? "PRESSED" : "RELEASED");
    Serial.print(", LED="); Serial.println(ledOn ? "ON" : "OFF");
  }
}

// QP4_HW_NoServo.ino — REAL PCI (D8/D9) + REAL Timer2 @1kHz (heartbeat + A0 sampling)
// No servo. LED12 = action, D13 = heartbeat. Serial: 115200

#include <avr/io.h>
#include <avr/interrupt.h>

const byte PIN_BTN_A   = 2;    // INT0
const byte PIN_BTN_B   = 8;    // PB0 / PCINT0
const byte PIN_BTN_C   = 9;    // PB1 / PCINT1
const byte PIN_LED_ACT = 12;   // action LED
const byte PIN_LED_HB  = 13;   // heartbeat LED (built-in)
const byte ADC_PIN     = A0;   // analog input (use A0 directly)

volatile bool    flagArmToggle   = false;
unsigned long    lastArmToggleMs = 0;
const unsigned long ARM_LOCKOUT_MS = 80;

volatile uint8_t pbLastMasked   = 0;
volatile uint8_t pcintPending   = 0;   // bit0 = B changed, bit1 = C changed
volatile uint8_t rawBLevel      = HIGH;
volatile uint8_t rawCLevel      = HIGH;

volatile bool flagBeat = false;       // heartbeat toggle request
volatile bool flagSampleA0 = false;   // analog sample request

ISR(PCINT0_vect) {
  uint8_t pb = PINB & (_BV(PB0) | _BV(PB1));
  uint8_t changed = (uint8_t)(pb ^ pbLastMasked);
  if (changed & _BV(PB0)) { pcintPending |= _BV(0); rawBLevel = (pb & _BV(PB0)) ? HIGH : LOW; }
  if (changed & _BV(PB1)) { pcintPending |= _BV(1); rawCLevel = (pb & _BV(PB1)) ? HIGH : LOW; }
  pbLastMasked = pb;
}

ISR(TIMER2_COMPA_vect) {
  static uint16_t beatDown   = 500;   // 500 ms -> 1 Hz toggle
  static uint16_t sampleDown = 100;   // 100 ms
  if (--beatDown   == 0) { beatDown   = 500; flagBeat     = true; }
  if (--sampleDown == 0) { sampleDown = 100; flagSampleA0 = true; }
}

void isrArmFalling() { flagArmToggle = true; }

bool armed = false;
bool stableB = HIGH, stableC = HIGH;
unsigned long lastDebB = 0, lastDebC = 0;
const unsigned long DEBOUNCE_MS = 20;

uint8_t mode = 0;                      // 0: armed&&(B||C) ; 1: armed&&(B&&C)
unsigned long comboStartMs = 0;
uint8_t comboWait = 0;                 // 0 none, 1 wait C, 2 wait B
const unsigned long COMBO_WINDOW_MS = 300;

int a0Raw = 0;

void setup() {
  pinMode(PIN_BTN_A,   INPUT_PULLUP);
  pinMode(PIN_BTN_B,   INPUT_PULLUP);
  pinMode(PIN_BTN_C,   INPUT_PULLUP);
  pinMode(PIN_LED_ACT, OUTPUT);
  pinMode(PIN_LED_HB,  OUTPUT);
  digitalWrite(PIN_LED_ACT, LOW);
  digitalWrite(PIN_LED_HB,  LOW);

  Serial.begin(115200);
  Serial.println(F("[QP4 HW] REAL PCI (D8/D9) + Timer2 @1kHz: heartbeat + A0"));

  // External interrupt (A = D2)
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_A), isrArmFalling, FALLING);

  // Enable PCINT for Port B (D8/D9)
  pbLastMasked = PINB & (_BV(PB0) | _BV(PB1));
  rawBLevel = (pbLastMasked & _BV(PB0)) ? HIGH : LOW;
  rawCLevel = (pbLastMasked & _BV(PB1)) ? HIGH : LOW;
  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1);
  PCIFR  |= _BV(PCIF0); // clear any pending

  // Timer2 CTC @ 1kHz: 16 MHz / (64 * (1 + 249)) = 1000 Hz
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);   // prescaler 64
  OCR2A  = 249;
  TIMSK2 = _BV(OCIE2A); // enable compare A interrupt
}

void handleArm() {
  if (!flagArmToggle) return;
  noInterrupts(); flagArmToggle = false; interrupts();
  unsigned long now = millis();
  if (now - lastArmToggleMs >= ARM_LOCKOUT_MS) {
    lastArmToggleMs = now;
    armed = !armed;
    Serial.print(F("INT0: ARMED -> ")); Serial.println(armed ? F("true") : F("false"));
  } else {
    Serial.println(F("INT0: toggle ignored (lockout)"));
  }
}

void processEdge(uint8_t pin, uint8_t level) {
  unsigned long now = millis();
  if (pin == 8) {
    if (now - lastDebB >= DEBOUNCE_MS) {
      lastDebB = now;
      stableB = (level == LOW) ? LOW : HIGH;
      Serial.print(F("PCI: pin=8 ")); Serial.println(stableB==LOW ? F("FALL(press)") : F("RISE(release)"));
      if (stableB == LOW) { comboStartMs = now; comboWait = 1; } // wait for C
    }
  } else if (pin == 9) {
    if (now - lastDebC >= DEBOUNCE_MS) {
      lastDebC = now;
      stableC = (level == LOW) ? LOW : HIGH;
      Serial.print(F("PCI: pin=9 ")); Serial.println(stableC==LOW ? F("FALL(press)") : F("RISE(release)"));
      if (stableC == LOW) { comboStartMs = now; comboWait = 2; } // wait for B
    }
  }
}

void handlePcint() {
  noInterrupts();
  uint8_t pending = pcintPending; pcintPending = 0;
  uint8_t bLevel = rawBLevel;
  uint8_t cLevel = rawCLevel;
  interrupts();

  if (pending & _BV(0)) processEdge(8, bLevel);
  if (pending & _BV(1)) processEdge(9, cLevel);
}

void handleCombo() {
  if (comboWait == 0) return;
  unsigned long now = millis();
  if (now - comboStartMs > COMBO_WINDOW_MS) { comboWait = 0; return; }
  if (comboWait == 1 && stableC == LOW) {
    mode ^= 1; comboWait = 0;
    Serial.print(F("COMBO: B->C (mode=")); Serial.print(mode); Serial.println(F(")"));
  } else if (comboWait == 2 && stableB == LOW) {
    mode ^= 1; comboWait = 0;
    Serial.print(F("COMBO: C->B (mode=")); Serial.print(mode); Serial.println(F(")"));
  }
}

void handleTimerTasks() {
  if (flagBeat) {
    flagBeat = false;
    digitalWrite(PIN_LED_HB, !digitalRead(PIN_LED_HB));
    Serial.println(F("TMR: beat"));
  }
  if (flagSampleA0) {
    flagSampleA0 = false;
    a0Raw = analogRead(ADC_PIN);
    Serial.print(F("TMR: A0=")); Serial.println(a0Raw);
  }
}

void driveActLed() {
  bool on = false;
  if (armed) {
    if (mode == 0) on = (stableB == LOW) || (stableC == LOW);
    else           on = (stableB == LOW) && (stableC == LOW);
  }
  digitalWrite(PIN_LED_ACT, on ? HIGH : LOW);
}

void periodicStatus() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 1000UL) {
    last = now;
    Serial.print(F("Status: armed=")); Serial.print(armed ? F("true") : F("false"));
    Serial.print(F(", B=")); Serial.print(stableB==LOW ? F("PRESSED") : F("RELEASED"));
    Serial.print(F(", C=")); Serial.print(stableC==LOW ? F("PRESSED") : F("RELEASED"));
    Serial.print(F(", mode=")); Serial.print(mode);
    Serial.print(F(", A0=")); Serial.print(a0Raw);
    Serial.print(F(", LED12=")); Serial.print(digitalRead(PIN_LED_ACT) ? F("ON") : F("OFF"));
    Serial.print(F(", HB13=")); Serial.println(digitalRead(PIN_LED_HB) ? F("ON") : F("OFF"));
  }
}

void loop() {
  handleArm();          // EXT INT (A)
  handlePcint();        // PCI events (B,C)
  handleCombo();        // grouped logic
  handleTimerTasks();   // Timer2-driven jobs
  driveActLed();        // actuator LED
  periodicStatus();     // 1s trace
}
// QP3_PCI_Hardware.ino — REAL Pin Change Interrupts on D8 & D9 (Port B).
// Use this on your physical Uno. Serial: 115200. Pins: A=D2, B=D8, C=D9, LED=D12 (or 13).

#include <avr/io.h>
#include <avr/interrupt.h>

const byte PIN_BTN_A = 2;   // D2 -> INT0
const byte PIN_BTN_B = 8;   // D8 -> PB0 / PCINT0
const byte PIN_BTN_C = 9;   // D9 -> PB1 / PCINT1
const byte PIN_LED   = 12;

// ---- A (external interrupt) ----
volatile bool flagArmToggle = false;
unsigned long lastArmToggleMs = 0;
const unsigned long ARM_LOCKOUT_MS = 80;
void isrArmFalling(){ flagArmToggle = true; }

// ---- PCI change flags/levels set by ISR ----
volatile uint8_t pbLastMasked = 0;
volatile uint8_t pcintPending = 0;   // bit0=PB0 changed, bit1=PB1 changed
volatile uint8_t rawBLevel    = HIGH;
volatile uint8_t rawCLevel    = HIGH;

// ---- Debounce & grouped logic ----
bool armed = false;
bool stableB = HIGH, stableC = HIGH;
unsigned long lastDebB = 0, lastDebC = 0;
const unsigned long DEBOUNCE_MS = 20;

uint8_t mode = 0; // 0: armed&&(B||C); 1: armed&&(B&&C)
unsigned long comboStartMs = 0;
uint8_t comboWait = 0;
const unsigned long COMBO_WINDOW_MS = 300;

void setup(){
  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_C, INPUT_PULLUP);
  pinMode(PIN_LED,   OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  Serial.println("[QP3 PCI] Real PCINT on D8 & D9 + grouped logic");

  attachInterrupt(digitalPinToInterrupt(PIN_BTN_A), isrArmFalling, FALLING);

  // Init PB snapshot
  pbLastMasked = PINB & (_BV(PB0) | _BV(PB1));
  rawBLevel = (pbLastMasked & _BV(PB0)) ? HIGH : LOW;
  rawCLevel = (pbLastMasked & _BV(PB1)) ? HIGH : LOW;

  // Enable PCINT on Port B (PCINT0..7). Unmask PCINT0 & PCINT1 only.
  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1);
  PCIFR  |= _BV(PCIF0); // clear any pending
}

ISR(PCINT0_vect){
  uint8_t pb = PINB & (_BV(PB0) | _BV(PB1));
  uint8_t changed = (uint8_t)(pb ^ pbLastMasked);
  if(changed & _BV(PB0)){ pcintPending |= _BV(0); rawBLevel = (pb & _BV(PB0)) ? HIGH : LOW; }
  if(changed & _BV(PB1)){ pcintPending |= _BV(1); rawCLevel = (pb & _BV(PB1)) ? HIGH : LOW; }
  pbLastMasked = pb;
}

void handleArm(){
  if(!flagArmToggle) return;
  noInterrupts(); flagArmToggle = false; interrupts();
  unsigned long now = millis();
  if(now - lastArmToggleMs >= ARM_LOCKOUT_MS){
    lastArmToggleMs = now;
    armed = !armed;
    Serial.print("INT0: ARMED -> "); Serial.println(armed ? "true" : "false");
  }else{
    Serial.println("INT0: toggle ignored (lockout)");
  }
}

void processEdge(uint8_t pin, uint8_t level){
  unsigned long now = millis();
  if(pin == 8){
    if(now - lastDebB >= DEBOUNCE_MS){
      lastDebB = now;
      stableB = (level == LOW) ? LOW : HIGH;
      Serial.print("PCI: pin=8 "); Serial.println(stableB==LOW ? "FALL(press)" : "RISE(release)");
      if(stableB == LOW){ comboStartMs = now; comboWait = 1; }
    }
  }else if(pin == 9){
    if(now - lastDebC >= DEBOUNCE_MS){
      lastDebC = now;
      stableC = (level == LOW) ? LOW : HIGH;
      Serial.print("PCI: pin=9 "); Serial.println(stableC==LOW ? "FALL(press)" : "RISE(release)");
      if(stableC == LOW){ comboStartMs = now; comboWait = 2; }
    }
  }
}

void handlePcint(){
  noInterrupts();
  uint8_t pending = pcintPending;
  pcintPending = 0;
  uint8_t bLevel = rawBLevel;
  uint8_t cLevel = rawCLevel;
  interrupts();

  if(pending & _BV(0)) processEdge(8, bLevel);
  if(pending & _BV(1)) processEdge(9, cLevel);
}

void handleCombo(){
  if(comboWait == 0) return;
  unsigned long now = millis();
  if(now - comboStartMs > COMBO_WINDOW_MS){ comboWait = 0; return; }
  if(comboWait == 1 && stableC == LOW){
    mode ^= 1; comboWait = 0;
    Serial.print("COMBO: B->C (mode="); Serial.print(mode); Serial.println(")");
  }else if(comboWait == 2 && stableB == LOW){
    mode ^= 1; comboWait = 0;
    Serial.print("COMBO: C->B (mode="); Serial.print(mode); Serial.println(")");
  }
}

void driveActuators(){
  bool led = false;
  if(armed){
    if(mode == 0) led = (stableB == LOW) || (stableC == LOW);
    else          led = (stableB == LOW) && (stableC == LOW);
  }
  digitalWrite(PIN_LED, led ? HIGH : LOW);
}

void periodicStatus(){
  static unsigned long last = 0;
  unsigned long now = millis();
  if(now - last >= 1000UL){
    last = now;
    Serial.print("Status: armed="); Serial.print(armed ? "true" : "false");
    Serial.print(", B="); Serial.print(stableB==LOW ? "PRESSED" : "RELEASED");
    Serial.print(", C="); Serial.print(stableC==LOW ? "PRESSED" : "RELEASED");
    Serial.print(", mode="); Serial.print(mode);
    Serial.print(", LED="); Serial.println(digitalRead(PIN_LED) ? "ON" : "OFF");
  }
}

void loop(){
  handleArm();     // INT0
  handlePcint();   // REAL PCI edges from ISR (hardware)
  handleCombo();
  driveActuators();
  periodicStatus();
}
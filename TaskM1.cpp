// QP5_HD_SlowShow.ino — HD+ demo-friendly build (Uno R3)
//
// Features:
// - REAL PCI on D8 (PB0/PCINT0) & D9 (PB1/PCINT1)
// - REAL Timer2 tick with adjustable "virtual periods"
// - EEPROM-backed config (soilThresh, comboWindowMs, defaultMode, defaultArmed)
// - Serial menu to view/set/save config
// - Watchdog (WDT) + reset-cause logging
// - Structured logs + VERBOSITY + on-the-fly rate control
//
// Pins (INPUT_PULLUP on switches):
//   A (arm) = D2 (EXT INT0, momentary to GND)
//   B       = D8 (slide center to D8, one end to GND)
//   C       = D9 (slide center to D9, one end to GND)
//   LED_ACT = D12 (220Ω -> LED anode, LED cathode -> GND)
//   LED_HB  = D13 (built-in heartbeat)
//   ADC     = A0  (soil sensor AOUT or potentiometer center)
//
// Serial: 115200

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <Arduino.h>

// ---------------- Pins ----------------
const uint8_t PIN_BTN_A   = 2;    // INT0 (arm)
const uint8_t PIN_BTN_B   = 8;    // PB0 / PCINT0
const uint8_t PIN_BTN_C   = 9;    // PB1 / PCINT1
const uint8_t PIN_LED_ACT = 12;   // action LED
const uint8_t PIN_LED_HB  = 13;   // heartbeat LED (built-in)
const uint8_t ADC_PIN     = A0;   // analog input

// ---------------- Persistent config ----------------
struct Config {
  uint16_t soilThresh;      // A0 threshold (0..1023)
  uint16_t comboWindowMs;   // B->C or C->B window to toggle mode
  uint8_t  defaultMode;     // 0=OR, 1=AND
  uint8_t  defaultArmed;    // 0=false, 1=true
  uint16_t crc;             // CRC16 of above fields
};

Config cfg;
const Config CFG_DEFAULT = { 600, 300, 0, 0, 0 };
const int EEPROM_ADDR = 0;

// CRC16 helpers
uint16_t crc16_update(uint16_t crc, uint8_t a){
  crc ^= a;
  for (uint8_t i=0; i<8; i++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  return crc;
}
uint16_t crc16_config(const Config &c){
  const uint8_t* p = (const uint8_t*)&c;
  size_t n = sizeof(Config) - sizeof(uint16_t);
  uint16_t crc = 0xFFFF;
  for (size_t i=0; i<n; i++) crc = crc16_update(crc, p[i]);
  return crc;
}
void eepromWriteConfig(const Config &c){
  Config tmp = c; tmp.crc = crc16_config(tmp);
  const uint8_t* p = (const uint8_t*)&tmp;
  for (size_t i=0; i<sizeof(Config); i++) EEPROM.update(EEPROM_ADDR + i, p[i]);
}
bool eepromReadConfig(Config &out){
  uint8_t* p = (uint8_t*)&out;
  for (size_t i=0; i<sizeof(Config); i++) p[i] = EEPROM.read(EEPROM_ADDR + i);
  return out.crc == crc16_config(out);
}

// ---------------- Runtime state ----------------
// A (external INT0)
volatile bool flagArmToggle = false;
unsigned long lastArmToggleMs = 0;
const unsigned long ARM_LOCKOUT_MS = 80;

// PCI edge capture from ISR
volatile uint8_t pbLastMasked = 0;
volatile uint8_t pcintPending = 0;   // bit0=B changed, bit1=C changed
volatile uint8_t rawBLevel    = HIGH;
volatile uint8_t rawCLevel    = HIGH;

// Timer2 flags (set by ISR)
volatile bool flagBeat     = false;  // heartbeat toggle request
volatile bool flagSampleA0 = false;  // A0 sample request

// Adjustable "virtual periods" (ms) updated from menu
volatile uint16_t beatReloadMs   = 1000; // default slow for demo
volatile uint16_t sampleReloadMs = 500;  // default slow for demo

// Verbosity & print decimation (to keep serial calm)
volatile uint8_t verbosity = 1;     // 0=quiet, 1=normal, 2=verbose
uint16_t beatPrintEvery    = 1;     // print every N beats
uint16_t samplePrintEvery  = 1;     // print every N samples
uint16_t beatPrintCount    = 0;
uint16_t samplePrintCount  = 0;

// Debounce & logic
bool armed = false;
bool stableB = HIGH, stableC = HIGH;
unsigned long lastDebB = 0, lastDebC = 0;
const unsigned long DEBOUNCE_MS = 20;

uint8_t mode = 0;                    // 0: armed&&(B||C), 1: armed&&(B&&C)
unsigned long comboStartMs = 0;
uint8_t comboWait = 0;               // 0 none, 1 wait C, 2 wait B

int a0Raw = 0;

// ---------------- Serial menu ----------------
// Commands (newline-terminated):
// H            = help
// S            = status
// T<val>       = set soil threshold           (e.g., T520)
// W<ms>        = set combo window (50..2000)  (e.g., W800)
// M0|M1        = set mode (0=OR,1=AND)
// A0|A1        = set default armed (0/1)
// P            = persist/save config to EEPROM
// R            = factory reset (defaults+save)
// X            = deliberate hang (watchdog demo)
// V0|V1|V2     = verbosity (0 quiet, 1 normal, 2 verbose)
// Z<ms>        = heartbeat period (100..5000) (e.g., Z1000)
// Y<ms>        = A0 sample period (20..5000)  (e.g., Y500)
char cmdBuf[32];
uint8_t cmdLen = 0;

void printHelp(){
  Serial.println(F("MENU,HELP"));
  Serial.println(F(" H    : help"));
  Serial.println(F(" S    : status"));
  Serial.println(F(" T### : set soil threshold (0..1023)"));
  Serial.println(F(" W### : set combo window ms (50..2000)"));
  Serial.println(F(" M0/1 : set mode (0=OR,1=AND)"));
  Serial.println(F(" A0/1 : set default armed (0/1)"));
  Serial.println(F(" P    : persist config to EEPROM"));
  Serial.println(F(" R    : factory reset (defaults + save)"));
  Serial.println(F(" X    : watchdog demo (deliberate hang)"));
  Serial.println(F(" V0/1/2 : verbosity (0 quiet, 1 normal, 2 verbose)"));
  Serial.println(F(" Z### : heartbeat period ms (100..5000)"));
  Serial.println(F(" Y### : sample period ms (20..5000)"));
}
void printConfigCsv(const char* tag){
  Serial.print(F("CFG,")); Serial.print(tag);
  Serial.print(F(",thresh="));   Serial.print(cfg.soilThresh);
  Serial.print(F(",comboMs="));  Serial.print(cfg.comboWindowMs);
  Serial.print(F(",defMode="));  Serial.print(cfg.defaultMode);
  Serial.print(F(",defArmed=")); Serial.print(cfg.defaultArmed);
  Serial.println();
}
void handleMenuLine(){
  if (!cmdLen) return;
  char c0 = cmdBuf[0];
  if (c0=='H'||c0=='h') { printHelp(); }
  else if (c0=='S'||c0=='s') {
    Serial.print(F("STA,armed=")); Serial.print(armed);
    Serial.print(F(",mode="));     Serial.print(mode);
    Serial.print(F(",B="));        Serial.print(stableB==LOW);
    Serial.print(F(",C="));        Serial.print(stableC==LOW);
    Serial.print(F(",A0="));       Serial.print(a0Raw);
    Serial.println();
    printConfigCsv("CUR");
  }
  else if (c0=='T'||c0=='t') {
    int v = atoi(&cmdBuf[1]);
    if (v>=0 && v<=1023) { cfg.soilThresh = (uint16_t)v; Serial.print(F("CFG,SET,thresh=")); Serial.println(cfg.soilThresh); }
    else Serial.println(F("ERR,bad_thresh"));
  }
  else if (c0=='W'||c0=='w') {
    int v = atoi(&cmdBuf[1]);
    if (v>=50 && v<=2000) { cfg.comboWindowMs = (uint16_t)v; Serial.print(F("CFG,SET,comboMs=")); Serial.println(cfg.comboWindowMs); }
    else Serial.println(F("ERR,bad_window"));
  }
  else if (c0=='M'||c0=='m') {
    if (cmdBuf[1]=='0' || cmdBuf[1]=='1') { mode = (cmdBuf[1]=='1'); cfg.defaultMode = mode; Serial.print(F("CFG,SET,mode=")); Serial.println(mode); }
    else Serial.println(F("ERR,bad_mode"));
  }
  else if (c0=='A'||c0=='a') {
    if (cmdBuf[1]=='0' || cmdBuf[1]=='1') { cfg.defaultArmed = (cmdBuf[1]=='1'); Serial.print(F("CFG,SET,defArmed=")); Serial.println(cfg.defaultArmed); }
    else Serial.println(F("ERR,bad_armed"));
  }
  else if (c0=='P'||c0=='p') { eepromWriteConfig(cfg); Serial.println(F("CFG,SAVE,OK")); }
  else if (c0=='R'||c0=='r') { cfg = CFG_DEFAULT; eepromWriteConfig(cfg); Serial.println(F("CFG,RESET,DEFAULTS_SAVED")); printConfigCsv("DEF"); }
  else if (c0=='X'||c0=='x') { Serial.println(F("WDT,DEMO,HANG_NOW")); while(1){} }
  else if (c0=='V'||c0=='v') {
    int v = atoi(&cmdBuf[1]);
    if (v>=0 && v<=2) { noInterrupts(); verbosity = (uint8_t)v; interrupts(); Serial.print(F("CFG,SET,verbosity=")); Serial.println((int)verbosity); }
    else Serial.println(F("ERR,bad_verbosity"));
  }
  else if (c0=='Z'||c0=='z') {
    int v = atoi(&cmdBuf[1]);
    if (v>=100 && v<=5000) { noInterrupts(); beatReloadMs = (uint16_t)v; interrupts(); Serial.print(F("CFG,SET,beatMs=")); Serial.println(beatReloadMs); }
    else Serial.println(F("ERR,bad_beat_ms"));
  }
  else if (c0=='Y'||c0=='y') {
    int v = atoi(&cmdBuf[1]);
    if (v>=20 && v<=5000) { noInterrupts(); sampleReloadMs = (uint16_t)v; interrupts(); Serial.print(F("CFG,SET,sampleMs=")); Serial.println(sampleReloadMs); }
    else Serial.println(F("ERR,bad_sample_ms"));
  }
  else { Serial.println(F("ERR,unknown_cmd")); }
  cmdLen = 0; // clear
}
void handleSerialMenu(){
  while (Serial.available()){
    char ch = Serial.read();
    if (ch=='\r' || ch=='\n') handleMenuLine();
    else if (cmdLen < sizeof(cmdBuf)-1) { cmdBuf[cmdLen++] = ch; cmdBuf[cmdLen] = 0; }
  }
}

// ---------------- ISRs ----------------
ISR(PCINT0_vect){
  uint8_t pb = PINB & (_BV(PB0) | _BV(PB1));
  uint8_t changed = (uint8_t)(pb ^ pbLastMasked);
  if (changed & _BV(PB0)) { pcintPending |= _BV(0); rawBLevel = (pb & _BV(PB0)) ? HIGH : LOW; }
  if (changed & _BV(PB1)) { pcintPending |= _BV(1); rawCLevel = (pb & _BV(PB1)) ? HIGH : LOW; }
  pbLastMasked = pb;
}

ISR(TIMER2_COMPA_vect){
  static uint16_t beatDown   = 1000; // start slow for demo feel
  static uint16_t sampleDown = 500;
  if (--beatDown == 0)   { beatDown = beatReloadMs;   flagBeat     = true; }
  if (--sampleDown == 0) { sampleDown = sampleReloadMs; flagSampleA0 = true; }
}

// ---------------- External INT0 ----------------
void isrArmFalling(){ flagArmToggle = true; }

// ---------------- Helpers ----------------
void printResetCause(uint8_t mcusr){
  if (mcusr & _BV(WDRF))  Serial.println(F("RST,WDT"));
  if (mcusr & _BV(BORF))  Serial.println(F("RST,BOD"));
  if (mcusr & _BV(EXTRF)) Serial.println(F("RST,EXT"));
  if (mcusr & _BV(PORF))  Serial.println(F("RST,POR"));
}

// ---------------- Setup/Loop ----------------
void setup(){
  // Capture reset cause, disable WDT early
  uint8_t mcusr_mirror = MCUSR; MCUSR = 0; wdt_disable();

  pinMode(PIN_BTN_A,   INPUT_PULLUP);
  pinMode(PIN_BTN_B,   INPUT_PULLUP);
  pinMode(PIN_BTN_C,   INPUT_PULLUP);
  pinMode(PIN_LED_ACT, OUTPUT);
  pinMode(PIN_LED_HB,  OUTPUT);
  digitalWrite(PIN_LED_ACT, LOW);
  digitalWrite(PIN_LED_HB,  LOW);

  Serial.begin(115200);
  // while (!Serial) {} // fine to omit on Uno
  Serial.println(F("HD+,[QP5] PCI+Timer2+EEPROM+WDT+Logs (SlowShow)"));
  printResetCause(mcusr_mirror);

  // Load config
  if (!eepromReadConfig(cfg)) { cfg = CFG_DEFAULT; eepromWriteConfig(cfg); Serial.println(F("CFG,EEPROM,BADCRC->DEFAULTS_SAVED")); }
  else                        { Serial.println(F("CFG,EEPROM,OK")); }
  printConfigCsv("BOOT");

  // Apply boot defaults
  mode  = cfg.defaultMode ? 1 : 0;
  armed = cfg.defaultArmed ? true : false;

  // External INT0 on D2 (press = FALLING)
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_A), isrArmFalling, FALLING);

  // PCI on Port B (PB0/D8, PB1/D9)
  pbLastMasked = PINB & (_BV(PB0) | _BV(PB1));
  rawBLevel = (pbLastMasked & _BV(PB0)) ? HIGH : LOW;
  rawCLevel = (pbLastMasked & _BV(PB1)) ? HIGH : LOW;
  PCICR  |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT0) | _BV(PCINT1);
  PCIFR  |= _BV(PCIF0);

  // Timer2 CTC @ 1kHz base tick: 16MHz/(64*(1+249)) = 1000 Hz
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);
  OCR2A  = 249;
  TIMSK2 = _BV(OCIE2A);

  // Enable watchdog (1s)
  wdt_enable(WDTO_1S);

  // Start in calm mode
  verbosity = 1;              // 0=quiet if you want ultra-calm
  beatReloadMs   = 1000;      // 1s heartbeat
  sampleReloadMs = 500;       // 500ms A0 sample
  beatPrintEvery   = 1;
  samplePrintEvery = 1;
}

void handleArm(){
  if (!flagArmToggle) return;
  noInterrupts(); flagArmToggle = false; interrupts();
  unsigned long now = millis();
  if (now - lastArmToggleMs >= ARM_LOCKOUT_MS) {
    lastArmToggleMs = now;
    armed = !armed;
    Serial.print(F("EVT,INT0,ARMED,")); Serial.println(armed ? F("1") : F("0"));
  } else {
    Serial.println(F("EVT,INT0,IGNORED,LOCKOUT"));
  }
}

void processEdge(uint8_t pin, uint8_t level){
  unsigned long now = millis();
  if (pin == 8) {
    if (now - lastDebB >= DEBOUNCE_MS) {
      lastDebB = now;
      stableB = (level == LOW) ? LOW : HIGH;
      Serial.print(F("EVT,PCI,8,")); Serial.println(stableB==LOW ? F("FALL") : F("RISE"));
      if (stableB == LOW) { comboStartMs = now; comboWait = 1; }
    }
  } else if (pin == 9) {
    if (now - lastDebC >= DEBOUNCE_MS) {
      lastDebC = now;
      stableC = (level == LOW) ? LOW : HIGH;
      Serial.print(F("EVT,PCI,9,")); Serial.println(stableC==LOW ? F("FALL") : F("RISE"));
      if (stableC == LOW) { comboStartMs = now; comboWait = 2; }
    }
  }
}

void handlePcint(){
  noInterrupts();
  uint8_t pending = pcintPending; pcintPending = 0;
  uint8_t bLevel = rawBLevel;
  uint8_t cLevel = rawCLevel;
  interrupts();

  if (pending & _BV(0)) processEdge(8, bLevel);
  if (pending & _BV(1)) processEdge(9, cLevel);
}

void handleCombo(){
  if (comboWait == 0) return;
  unsigned long now = millis();
  if (now - comboStartMs > cfg.comboWindowMs) { comboWait = 0; return; }
  if (comboWait == 1 && stableC == LOW) { mode ^= 1; comboWait = 0; Serial.print(F("EVT,COMBO,B2C,mode=")); Serial.println(mode); }
  else if (comboWait == 2 && stableB == LOW) { mode ^= 1; comboWait = 0; Serial.print(F("EVT,COMBO,C2B,mode=")); Serial.println(mode); }
}

void handleTimerTasks(){
  if (flagBeat){
    flagBeat = false;
    digitalWrite(PIN_LED_HB, !digitalRead(PIN_LED_HB)); // always toggle LED
    if (verbosity >= 1){
      if (++beatPrintCount >= beatPrintEvery){
        beatPrintCount = 0;
        Serial.println(F("TMR,BEAT"));
      }
    }
  }
  if (flagSampleA0){
    flagSampleA0 = false;
    a0Raw = analogRead(ADC_PIN); // always sample
    if (verbosity >= 1){
      if (verbosity >= 2 || ++samplePrintCount >= samplePrintEvery){
        samplePrintCount = 0;
        Serial.print(F("ANA,A0,")); Serial.println(a0Raw);
      }
    }
  }
}

void driveActLed(){
  bool on = false;
  if (armed){
    if (mode==0) on = (stableB==LOW) || (stableC==LOW);
    else         on = (stableB==LOW) && (stableC==LOW);
  }
  digitalWrite(PIN_LED_ACT, on ? HIGH : LOW);
}

void periodicStatus(){
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 1000UL){
    last = now;
    Serial.print(F("STAT,armed=")); Serial.print(armed);
    Serial.print(F(",mode="));       Serial.print(mode);
    Serial.print(F(",B="));          Serial.print(stableB==LOW);
    Serial.print(F(",C="));          Serial.print(stableC==LOW);
    Serial.print(F(",A0="));         Serial.print(a0Raw);
    Serial.print(F(",LED12="));      Serial.print(digitalRead(PIN_LED_ACT));
    Serial.print(F(",HB13="));       Serial.print(digitalRead(PIN_LED_HB));
    Serial.println();
  }
}

void loop(){
  wdt_reset();          // feed watchdog

  handleSerialMenu();   // non-blocking
  handleArm();          // A (EXT INT flag)
  handlePcint();        // B/C (PCI flags)
  handleCombo();        // grouped logic
  handleTimerTasks();   // time-based jobs
  driveActLed();        // actuator LED
  periodicStatus();     // 1 s status
}
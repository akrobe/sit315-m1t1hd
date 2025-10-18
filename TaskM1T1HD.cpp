// QP1_Baseline.ino — polling button on D2, LED on D13, Serial 115200
// Board: Arduino Uno R3 (A000066)

#include <Arduino.h>

constexpr uint8_t PIN_BTN_A = 2;            // D2 (INPUT_PULLUP), pressed => LOW
constexpr uint8_t PIN_LED   = 12;  // D13 built-in LED (tiny "L")

bool lastBtnA = HIGH;            // released by default with INPUT_PULLUP
unsigned long lastPrintMs = 0;

void setup() {
  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Serial.begin(115200);
  while (!Serial) {}  // wait for Serial on some hosts

  Serial.println("[QP1] Baseline started (polling).");
  Serial.println("Wiring: Button A between D2 and GND (bridging trench). Press => LED ON, Release => LED OFF.");
}

void loop() {
  // Sense
  bool btnA = digitalRead(PIN_BTN_A); // HIGH=released, LOW=pressed

  // Think: edge logging
  if (btnA != lastBtnA) {
    lastBtnA = btnA;
    if (btnA == LOW) {
      Serial.println("Button A PRESSED");
      // Act
      digitalWrite(PIN_LED, HIGH);
    } else {
      Serial.println("Button A RELEASED");
      digitalWrite(PIN_LED, LOW);
    }
  }

  // 1s status line (helps for screenshots)
  unsigned long now = millis();
  if (now - lastPrintMs >= 1000UL) {
    lastPrintMs = now;
    Serial.print("Status: ButtonA=");
    Serial.print((btnA == LOW) ? "PRESSED" : "RELEASED");
    Serial.print(", LED=");
    Serial.println(digitalRead(PIN_LED) ? "ON" : "OFF");
  }
}
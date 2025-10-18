// D2_Sanity_Test.ino — GOLD TEST: D2 sanity with INPUT_PULLUP + LED mirror
#include <Arduino.h>
void setup() {
  pinMode(2, INPUT_PULLUP);            // D2 idles HIGH (1)
  pinMode(LED_BUILTIN, OUTPUT);        // D13 LED mirrors the state
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("D2 sanity test: Release=1, Press/Short to GND=0");
}
void loop() {
  int v = digitalRead(2);
  Serial.print("READ: "); Serial.println(v);
  digitalWrite(LED_BUILTIN, v ? HIGH : LOW);  // 1 -> LED ON, 0 -> LED OFF
  delay(150);
}
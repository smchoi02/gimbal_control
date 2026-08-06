// I2C bus scanner for the tracker's shared bus.
//
// Use this when several I2C sensors fail at once. It separates three cases
// that the main firmware's pass/fail log cannot:
//   - nothing answers        -> wiring, power, or pull-ups
//   - some answer            -> the missing device alone is at fault
//   - the bus reads as stuck -> SDA or SCL held low
//
// Expected devices on this build:
//   0x42  MAX-M10S GPS
//   0x47  BMP581      (0x46 when the address pin is low)
//   0x4A  BNO085      (0x4B when ADR is high)

#include <Arduino.h>
#include <Wire.h>

// Some cores name the bus pins differently; fall back to the A4/A5 pair.
#ifndef PIN_WIRE_SDA
#define PIN_WIRE_SDA SDA
#endif
#ifndef PIN_WIRE_SCL
#define PIN_WIRE_SCL SCL
#endif

const uint32_t USB_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 400000;
const uint32_t SCAN_PERIOD_MS = 2000;

struct Known {
  uint8_t address;
  const char* name;
};

const Known KNOWN[] = {
  {0x42, "MAX-M10S GPS"},
  {0x46, "BMP581 (primary)"},
  {0x47, "BMP581 (secondary)"},
  {0x4A, "BNO085"},
  {0x4B, "BNO085 (ADR high)"},
};
const size_t KNOWN_COUNT = sizeof(KNOWN) / sizeof(KNOWN[0]);

const char* nameFor(uint8_t address) {
  for (size_t i = 0; i < KNOWN_COUNT; i++) {
    if (KNOWN[i].address == address) return KNOWN[i].name;
  }
  return "";
}

// Both lines idle high through their pull-ups. A line stuck low means a
// wiring fault or a device holding the bus, and no address will ever answer.
void reportIdleLevels() {
  pinMode(PIN_WIRE_SDA, INPUT);
  pinMode(PIN_WIRE_SCL, INPUT);
  delayMicroseconds(50);
  const int sda = digitalRead(PIN_WIRE_SDA);
  const int scl = digitalRead(PIN_WIRE_SCL);

  Serial.print(F("idle SDA="));
  Serial.print(sda == HIGH ? F("HIGH") : F("LOW"));
  Serial.print(F(" SCL="));
  Serial.print(scl == HIGH ? F("HIGH") : F("LOW"));
  if (sda == LOW || scl == LOW) {
    Serial.println(F("   <-- stuck low: nothing can answer until this is fixed"));
  } else {
    Serial.println(F("   (both idle high: pull-ups present)"));
  }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
}

void setup() {
  Serial.begin(USB_BAUD);
  const uint32_t start = millis();
  while (!Serial && millis() - start < 5000) {}

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Serial.println();
  Serial.println(F("=== I2C scan ==="));
  Serial.print(F("SDA pin ")); Serial.print(PIN_WIRE_SDA);
  Serial.print(F(", SCL pin ")); Serial.print(PIN_WIRE_SCL);
  Serial.print(F(", clock ")); Serial.print(I2C_CLOCK_HZ);
  Serial.println(F(" Hz"));
}

void loop() {
  reportIdleLevels();

  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission();
    if (error == 0) {
      found++;
      Serial.print(F("  0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      const char* name = nameFor(address);
      if (name[0]) { Serial.print(F("  <- ")); Serial.print(name); }
      Serial.println();
    }
  }

  Serial.print(F("total "));
  Serial.print(found);
  Serial.println(F(" device(s)"));

  if (found == 0) {
    Serial.println(F("Nothing answered. Check 3.3V and GND to every sensor,"));
    Serial.println(F("SDA/SCL not swapped, and 4.7k pull-ups to 3.3V."));
    Serial.println(F("Never power these sensors from 5V."));
  }

  Serial.println();
  delay(SCAN_PERIOD_MS);
}

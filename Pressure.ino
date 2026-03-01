#include <Wire.h>
#include <Adafruit_MPRLS.h>

// Adafruit_MPRLS(resetPin, eocPin)
// If you are NOT wiring reset/eoc, pass -1, -1.
Adafruit_MPRLS mprls(-1, -1);

static const uint32_t BAUD = 115200;
static const uint32_t SAMPLE_PERIOD_MS = 50;  // ~20 Hz; adjust as desired

void setup() {
  Serial.begin(BAUD);
  delay(200);

  Wire.begin();  // ESP32 default SDA/SCL pins; change with Wire.begin(SDA, SCL) if needed

  if (!mprls.begin()) {
    Serial.println("ERROR: MPRLS not found. Check wiring/I2C address.");
    while (true) { delay(1000); }
  }

  Serial.println("READY");
  Serial.println("FORMAT: millis,pressure_hpa");
}

void loop() {
  const uint32_t ms = millis();
  // Adafruit_MPRLS::readPressure() returns pressure in hPa
  const float p_hpa = mprls.readPressure();

  Serial.print(ms);
  Serial.print(",");
  Serial.println(p_hpa, 3);

  delay(SAMPLE_PERIOD_MS);
}

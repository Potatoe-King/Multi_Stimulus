#include <Arduino.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// --- Panel configuration ---
static const uint16_t PANEL_W = 64;
static const uint16_t PANEL_H = 64;
static const uint8_t  CHAIN_LEN = 1;   // one 64x64 panel

static const uint32_t BAUD = 115200;

MatrixPanel_I2S_DMA *dma_display = nullptr;
bool led_on = false;

void set_panel_state(bool on) {
  led_on = on;

  if (!dma_display) return;

  if (led_on) {
    dma_display->setBrightness8(255); // 0..255
    uint16_t white = dma_display->color565(255, 255, 255);
    dma_display->fillScreen(white);
  } else {
    dma_display->fillScreen(0);
  }
  dma_display->flipDMABuffer(); // if double buffering is enabled
}

void setup_panel() {
  HUB75_I2S_CFG mxconfig(PANEL_W, PANEL_H, CHAIN_LEN);

  // If you have custom pin mapping, set it here (otherwise default mapping is used).
  // Example (varies by your wiring and library fork):
  // mxconfig.gpio.r1 = 25; mxconfig.gpio.g1 = 26; mxconfig.gpio.b1 = 27; ...
  // mxconfig.gpio.a = 23; mxconfig.gpio.b = 19; mxconfig.gpio.c = 5; mxconfig.gpio.d = 17; mxconfig.gpio.e = 18;
  // mxconfig.gpio.lat = 4; mxconfig.gpio.oe = 15; mxconfig.gpio.clk = 16;

  mxconfig.double_buff = true;     // smoother updates
  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_20M; // typical stable speed

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->clearScreen();

  set_panel_state(false);
}

String readLineBlocking() {
  String s;
  while (true) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\n') return s;
      if (c != '\r') s += c;
    }
    delay(1);
  }
}

void setup() {
  Serial.begin(BAUD);
  delay(200);

  setup_panel();

  Serial.println("READY");
  Serial.println("Commands: ON | OFF");
}

void loop() {
  if (Serial.available() <= 0) {
    delay(5);
    return;
  }

  String cmd = readLineBlocking();
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "ON") {
    set_panel_state(true);
    Serial.println("OK ON");
  } else if (cmd == "OFF") {
    set_panel_state(false);
    Serial.println("OK OFF");
  } else {
    Serial.println("ERR");
  }
}

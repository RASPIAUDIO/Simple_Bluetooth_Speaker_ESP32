#include <Adafruit_NeoPixel.h>
#include "esp_system.h"
#include "esp_attr.h"

#define NEOPIXEL_PIN 22
#define NUMPIXELS 1
#define BLINK_DELAY_MS 500

RTC_DATA_ATTR bool g_softResetDone = false;

Adafruit_NeoPixel pixel(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static void handleFirstBootReset() {
  const esp_reset_reason_t reason = esp_reset_reason();

  if (reason == ESP_RST_SW || reason == ESP_RST_DEEPSLEEP) {
    // Already coming from a software/deep-sleep reset; skip to avoid looping.
    g_softResetDone = true;
    return;
  }

  if (!g_softResetDone && reason == ESP_RST_POWERON) {
    g_softResetDone = true;
    delay(100);
    esp_restart();
  }

  // Any other reason (brownout, watchdog, etc.) => do not trigger another reset.
  g_softResetDone = true;
}

void setup() {
  handleFirstBootReset();

  pixel.begin();
  pixel.setBrightness(64);
  pixel.clear();
  pixel.show();
}

void loop() {
  static bool ledOn = false;

  pixel.setPixelColor(0, ledOn ? pixel.Color(0, 0, 0) : pixel.Color(32, 32, 32));
  pixel.show();

  ledOn = !ledOn;
  delay(BLINK_DELAY_MS);
}

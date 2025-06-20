/***************************************************************************
 *  MUSE  LUXE / PROTO – Bluetooth-speaker firmware (single file)          *
 *                                                                         *
 *  >>>  BOARD SELECTION  <<<                                              *
 *      - LUXE  : keep    #define BOARD_LUXE                               *
 *      - PROTO : uncomment #define BOARD_PROTO and comment BOARD_LUXE     *
 ***************************************************************************/

#define BOARD_PROTO            // ← enable for the PROTO board
// #define BOARD_LUXE          // ← enable for the LUXE board (default)

/* --------------------------------------------------------------------- */
/*  Sanity checks / mutual exclusion                                      */
#if defined(BOARD_LUXE) && defined(BOARD_PROTO)
#error "Select either BOARD_LUXE or BOARD_PROTO – not both."
#endif
#if !defined(BOARD_LUXE) && !defined(BOARD_PROTO)
#define BOARD_LUXE            // safety: default to LUXE
#endif

/*  Feature flags                                                         */
#if defined(BOARD_LUXE)
#define HAS_CODEC 1           // ES8388 present
#define HAS_JACK_DETECT 1
#else                         // PROTO
#define GAIN 23
#define HAS_CODEC 0           // pure I²S
#define HAS_JACK_DETECT 0
#endif
/* --------------------------------------------------------------------- */

/* ======================   INCLUDES   ================================== */
#include "museWrover.h"       // already contains the ES8388 class
#if HAS_CODEC
/*  No extra header needed: ES8388 is in museWrover.h                     */
#endif

#include "BluetoothA2DPSink.h"   // https://github.com/pschatzmann/ESP32-A2DP
#include "A2DPVolumeControl.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include <Adafruit_NeoPixel.h>
#include "LittleFS.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

/* ======================   GLOBAL VARIABLES   ========================= */
bool BPause = false;
int volume  = 75;             // range 0…100
#define maxVol 100

I2SClass            i2s;
BluetoothA2DPSink   a2dp_sink(i2s);

uint8_t  mac[6];
char     macStr[20];
char     dev_name[30];
bool     jack = false;        // headphones plugged in?
uint8_t* b    = nullptr;

/*  Battery LED strip */
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* ======================   FUNCTIONS   ================================= */

/* ----------- BATTERY TASK -------------------------------------------- */
#define NGREEN  2300
#define NYELLOW 1800
#define ADC_BAT 33
static void battery(void*) {
  for (;;) {
    int val = analogRead(ADC_BAT);
    Serial.printf("Battery : %d\n", val);

    if      (val < NYELLOW) pixels.setPixelColor(0, pixels.Color(255, 0,   0));   // red
    else if (val > NGREEN)  pixels.setPixelColor(0, pixels.Color(0,   255, 0));   // green
    else                    pixels.setPixelColor(0, pixels.Color(255, 255, 0));  // yellow

    pixels.show();
    vTaskDelay(pdMS_TO_TICKS(10'000));   // update every 10 s
  }
}

/* ======================   SETUP   ===================================== */
void setup() {
  Serial.begin(115200);

  /* 60-s watchdog ------------------------------------------------------ */
  esp_task_wdt_config_t wdt = {
    .timeout_ms      = 60 * 1000,
    .idle_core_mask  = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic   = true
  };
  esp_task_wdt_init(&wdt);

  /* GPIO --------------------------------------------------------------- */
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);

  gpio_reset_pin(BUTTON_VOL_PLUS);
  gpio_set_direction(BUTTON_VOL_PLUS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_VOL_PLUS, GPIO_PULLUP_ONLY);

  gpio_reset_pin(BUTTON_VOL_MINUS);
  gpio_set_direction(BUTTON_VOL_MINUS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_VOL_MINUS, GPIO_PULLUP_ONLY);

  gpio_reset_pin(BUTTON_PAUSE);
  gpio_set_direction(BUTTON_PAUSE, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PAUSE, GPIO_PULLUP_ONLY);

#if HAS_JACK_DETECT
  gpio_reset_pin(Jack_Detect);
  gpio_set_direction(Jack_Detect, GPIO_MODE_INPUT);
  gpio_set_pull_mode(Jack_Detect, GPIO_PULLUP_ONLY);
#endif

  /* Factory-test mode: hold all three buttons at boot ----------------- */
  if (!gpio_get_level(BUTTON_VOL_PLUS) &&
      !gpio_get_level(BUTTON_VOL_MINUS) &&
      !gpio_get_level(BUTTON_PAUSE)) {
    Serial.println("Factory test mode");
    const esp_partition_t* p = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr);
    if (p) {
      esp_ota_set_boot_partition(p);
      esp_restart();
    }
  }

#if HAS_CODEC
  /* ES8388 codec initialization --------------------------------------- */
  Serial.printf("ES8388 … ");
  while (!es.begin(IIC_DATA, IIC_CLK)) {
    Serial.println("fail");
    delay(1000);
  }
  Serial.println("OK");
  es.volume(ES8388::ES_MAIN, maxVol);
  es.volume(ES8388::ES_OUT1, volume);
  es.mute  (ES8388::ES_MAIN, false);
  es.mute  (ES8388::ES_OUT1, false);


    i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("I2S init fail");
    while (1) delay(1000);
  }
#else
  Serial.println("PROTO build – no codec, direct I²S output");
  /* Gain pin for PROTO */
  pinMode(GAIN, OUTPUT);
  digitalWrite(GAIN, LOW);
  a2dp_sink.set_volume(maxVol);

    i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_MONO, I2S_STD_SLOT_BOTH)) {
    Serial.println("I2S init fail");
    while (1) delay(1000);
  }
#endif

  /* Bluetooth device name based on MAC address ------------------------ */
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(macStr, sizeof(macStr), "-%02X%02X%02X", mac[3], mac[4], mac[5]);


#if BOARD_LUXE
  strcpy(dev_name, "LUXE");
#else
  strcpy(dev_name, "PROTO");

#endif
  
  
  
  
  strcat(dev_name, macStr);
  Serial.printf("Device : %s\n", dev_name);
  a2dp_sink.start(dev_name);

  /* LittleFS + startup sound ------------------------------------------ */
  if (!LittleFS.begin()) {
    Serial.println("LittleFS init fail");
    while (1) delay(1000);
  }



  b = (uint8_t*)ps_malloc(160000);
  File f = LittleFS.open("/bluetooth.wav", FILE_READ);
  if (f) {
    f.read(b, 115000);
    f.close();
    i2s.playWAV(b, 115000);
  }
  free(b);

  i2s.end();   // reopen for A2DP streaming
  i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);

  /* Battery task pinned to core-0 ------------------------------------- */
  xTaskCreatePinnedToCore(battery, "battery", 4096, nullptr, 5, nullptr, 0);
}

/* ======================   LOOP   ====================================== */
void loop() {
  /* No audio stream → yield a bit of CPU time ------------------------- */
  if (a2dp_sink.get_audio_state() != ESP_A2D_AUDIO_STATE_STARTED)
    delay(1000);

  /* ---------- Play / Pause ------------------------------------------- */
  if (!gpio_get_level(BUTTON_PAUSE)) {
    while (!gpio_get_level(BUTTON_PAUSE)) delay(10);

    if (!BPause) {                // switch to pause
#if HAS_CODEC
      es.mute(ES8388::ES_MAIN, true);
      if (jack) {
        es.volume(ES8388::ES_OUT2, 0);
        es.mute  (ES8388::ES_OUT2, true);
      } else {
        es.volume(ES8388::ES_OUT1, 0);
        es.mute  (ES8388::ES_OUT1, true);
      }
#endif
      a2dp_sink.pause();
    } else {                      // resume playback
#if HAS_CODEC
      es.mute(ES8388::ES_MAIN, false);
      if (jack) {
        es.volume(ES8388::ES_OUT2, volume);
        es.mute  (ES8388::ES_OUT2, false);
      } else {
        es.volume(ES8388::ES_OUT1, volume);
        es.mute  (ES8388::ES_OUT1, false);
      }
#endif
      a2dp_sink.play();
    }
    BPause = !BPause;
  }

  /* ---------- Volume + ----------------------------------------------- */
  if (!gpio_get_level(BUTTON_VOL_PLUS)) {
    while (!gpio_get_level(BUTTON_VOL_PLUS)) delay(10);
    volume = min(volume + 3, maxVol);
    Serial.printf("v=%d\n", volume);
#if HAS_CODEC
    es.volume(ES8388::ES_MAIN, volume < 6 ? volume : maxVol);
    if (jack) es.volume(ES8388::ES_OUT2, volume);
    else      es.volume(ES8388::ES_OUT1, volume);
#else
    a2dp_sink.set_volume(volume);
#endif
  }

  /* ---------- Volume - ----------------------------------------------- */
  if (!gpio_get_level(BUTTON_VOL_MINUS)) {
    while (!gpio_get_level(BUTTON_VOL_MINUS)) delay(10);
    volume = max(volume - 3, 0);
    Serial.printf("v=%d\n", volume);
#if HAS_CODEC
    es.volume(ES8388::ES_MAIN, volume < 6 ? volume : maxVol);
    if (jack) es.volume(ES8388::ES_OUT2, volume);
    else      es.volume(ES8388::ES_OUT1, volume);
#else
    a2dp_sink.set_volume(volume);
#endif
  }

#if HAS_JACK_DETECT
  /* ---------- Jack detection ----------------------------------------- */
  if (!gpio_get_level(Jack_Detect)) {      // headphones plugged in
    if (!jack) {
      es.volume(ES8388::ES_OUT2, volume);
      es.select_out2();
      gpio_set_level(GPIO_PA_EN, 0);       // amplifier off
    }
    jack = true;
  } else {                                 // headphones unplugged
    if (jack) {
      es.volume(ES8388::ES_OUT1, volume);
      es.select_out1();
      gpio_set_level(GPIO_PA_EN, 1);       // amplifier on
    }
    jack = false;
  }
#endif
}
